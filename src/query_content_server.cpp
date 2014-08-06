/*
 TODO: needs revision
   - check wether string input is individual or class (+ of type container)
   a) for individual
   - check content in database
   -(ok) check position in database, and wether visibility in real world
   -(ok) check color in image
   - compare to content specified in database
   - if not given look for filling action

   b) if class, (expect visibility)
   - check next incoming perception for color content
   - compare to specification in database
   - if not given look for filling action
*/

#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>
#include <json_prolog/prolog.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <perception_ar_kinect/QueryContentAction.h>
#include <perception_ar_kinect/GetDrinkColorAction.h>

#include <ar_pose/ARMarkers.h>
#include <ar_pose/ARMarker.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>

#include "knowrobMapping.h"

using namespace std;
using namespace json_prolog;

class QueryContentAction
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<perception_ar_kinect::QueryContentAction> as_;
  std::string action_name_;
  perception_ar_kinect::QueryContentResult result_;
  perception_ar_kinect::QueryContentFeedback feedback_;
  perception_ar_kinect::QueryContentGoalConstPtr goal_;

  ros::Subscriber sub_;
  Prolog pl_;
  bool observe_;

  bool db_content_av_;
  bool color_av_; 
  string db_content_;
  string color_;
  string objInst_;
  

public:
  QueryContentAction(std::string name) :
    as_(nh_,name, false), action_name_(name),
    observe_(false), db_content_av_(false), color_av_(false)
  {
    as_.registerGoalCallback(boost::bind(&QueryContentAction::goalCB, this));
    sub_ = nh_.subscribe("/ar_pose_markers", 1, &QueryContentAction::markerCallback ,this); 
    as_.start();
  } 

  void goalCB()
  {
    goal_ = as_.acceptNewGoal();

    int r = instOrClass(goal_->container, "Container");
    if(r == 0) { //input param is known individual
      objInst_ = goal_->container;
    }
    else if(r == 1) { // input param is a class
      observe_ = true; //activate MarkerCallback
      ros::Time begin = ros::Time::now();
      ros::Duration d(0.0);
      ros::Duration f(3.0);
      // wait till observe false or certain time passed
      while(observe_ && (d < f)) {
        d = ros::Time::now() - begin;
      }
      if (observe_)
      {
        observe_ = false;
        stringstream f;
        f << "Object of class " << goal_->container << " not visible";
        feedback_.feedback = f.str();
        as_.publishFeedback(feedback_);
        as_.setAborted();
      }
    } 
    else {
      as_.setAborted();
    }
    
    //query recorded content from database 
    if(getContentFromDatabase(objInst_, db_content_)) {
      db_content_av_ = true; 
    }
    //query pose from database and look for content color in image
    geometry_msgs::PoseStamped pose;
    if(getPoseOfObject(objInst_, pose)) {
      if(callGetDrinkColorAction(pose, color_)) {
        color_av_ = true; 
      }
    }

    //identify pose by observed color and database recordings
    if(identifyContent()) {
      as_.setSucceeded(result_);
    }
    else {
      std::cout << "identifyContent failed" << std::endl;
      as_.setAborted();
    }
  }


private:
  /**
   * ARMarker Callback, activated if knowrob class is passed as goal 
   *
   * finds marker of the same class as specified by goal
   * and identifies the corresponding object instance in database
   */
  void markerCallback(const ar_pose::ARMarkers& markers)
  {
    if(!as_.isActive() || !observe_){
      return;
    }
    feedback_.feedback = "markerCallback active";
    as_.publishFeedback(feedback_);
    const std::vector<ar_pose::ARMarker>& objects = markers.markers; 
    for (const ar_pose::ARMarker& obj : objects)
    {
      string type = knowrob_mapping::markerToObjClass(obj.id);
      if(type.compare(goal_->container))
      {
        geometry_msgs::PoseStamped pose;
        pose.pose = obj.pose.pose;
        pose.header = obj.header;
        objInst_ = knowrob_mapping::findObjInst(type, pose); 
        observe_ = false;
      }
    }
  }


  /**
   * identifies the content of objInst_ based on the observed color and
   * the information given in the database 
   *
   * TODO: detailed Description
   *
   */
  bool identifyContent()
  { 
   //content recorded in database and color observed in image 
    if(db_content_av_ && color_av_) {
      stringstream q;
      q << "owl_has(knowrob:'" << db_content_ << "', knowrob:mainColorOfObject, knowrob'" << color_ << "')";
      PrologBindings qr = pl_.once(q.str());
      if (qr.begin() != qr.end()) { // content and color match
        feedback_.feedback = "Content recorded in database and observed color match";
        as_.publishFeedback(feedback_);
        result_.content = db_content_; 
        return true;
      }
      else {
        // regard content information as expired and delete it from database
        db_content_av_ = false;
        q.str("");
        q << "rdf_retractall(knowrob:'" << objInst_ << "', knowrob:contains, _)";
        PrologBindings qr = pl_.once(q.str());
      }
    }

    //content color could be observed in image and content information in database is not given or expired
    if(color_av_ && !db_content_av_)
    {
      if(color_.compare("none"))
      {
        result_.content = "empty";
        return true;
      }
      stringstream q;
      //query for general content of container class
      q << "owl_has(knowrob:'" << objInst_ << "', rdf:type, _ObjClass),"
        << "class_properties(_ObjClass, knowrob:contains, _ContClass),"
      //find content subclasses with correct properties
        << "subclass_by_prop_val(_ContClass, knowrob:mainColorOfObject, knowrob:'" << color_ << "', Obj)";
      PrologQueryProxy qp = pl_.query(q.str());
      PrologQueryProxy::iterator it = qp.begin(); it++;
      if(it == qp.end()) //single solution
      {
        //content can be clearly identified by properties/color only
        PrologBindings qr = *qp.begin();
        result_.content = qr["Obj"].toString();
        //TODO: create instance and insert into database
        return true;
      } 
      //else find actions which lead to content change
      q << ", find_cause_of_appearance(knowrob:'" << objInst_ << "', knowrob:contains, Obj, _PRS),"
        << "member(_Pair, _PRS), pairs_keys_values([_Pair], [Action], [ObjActOn]),";
      stringstream q_copy;
      q_copy << q.str();

      //check for action instances working on objects with correct properties
      q << "object_possibly_available(ObjActOn, ObjInst),"
        << "find_latest_action_inst(Action, [ObjInst], [], ['http://ias.cs.tum.edu/kb/knowrob.owl#" << objInst_ << "'], ActionInst)";
      qp = pl_.query(q.str());
      for(it = qp.begin(); it != qp.end(); it++)
      {
        PrologBindings qr = *it;
        result_.content = qr["ObjInst"].toString(); 
        stringstream f;
        f << "Content of " << objInst_ << ", is same as content of " << result_.content << ".";
        feedback_.feedback = f.str();
        as_.publishFeedback(feedback_);
        return true;
        //TODO: in case of more than one solution compare timestamps
      }

      //TODO: find content based on the availability of sources 
         
      ROS_INFO("[query_content_server]end of identifyContent()");
      return true;
    }

    //content recorded in database
    else if(db_content_av_) {
      feedback_.feedback = "Object is not visible, but content was recorded in the database previously";
      as_.publishFeedback(feedback_);
      result_.content = db_content_; 
      return true;
    }
    else {
      feedback_.feedback = "Object is neither visible nor is its content recorded in the database";
      as_.publishFeedback(feedback_);
      result_.content = "unknown";
      return true;
    }
  }


  /**
   * checks wether an object label is a known individual or class in knowrob 
   *
   * @param obj    object label to be analysed 
   * @param supcl  superclass of the object 
   *
   * @return       0: obj is individual of superclass
   *               1: obj is subclass of superclass
   *               >1: obj is none of the above
   */ 
  int instOrClass(string obj, string supcl)
  {
    stringstream q;
    q << "owl_individual_of(knowrob:'" << obj << "', knowrob:'" << supcl << "')";
    try {
      PrologQueryProxy qr = pl_.query(q.str());
      if (qr.begin() != qr.end())
        return 0;
      else {
        q.str("");
        q << "owl_subclass_of(knowrob:'" << obj << "', knowrob:'" << supcl << "')";
        PrologQueryProxy qr = pl_.query(q.str());
        if (qr.begin() != qr.end())
          return 1; 
        else {
          feedback_.feedback = "Input is neither a known individual of type container, nor a container class";
          as_.publishFeedback(feedback_);
          return 2;
        }
      }
    } catch (PrologQueryProxy::QueryError ex) {
      ROS_ERROR("[query_content_server]%s", ex.what());
      return 2;
    }
  }

  /**
   * querys the last detection of an object and returns its pose 
   *
   * @param objInst  Name of the object instance in knowrob
   * @param pose     Last recorded pose of the object 
   */ 
  bool getPoseOfObject(string objInst, geometry_msgs::PoseStamped pose)
  {
    stringstream q;
    q << "current_object_pose(knowrob:'" << objInst << "', [M00,M01,M02,M03,M10,M11,M12,M13,M20,M21,M22,M23,M30,M31,M32,M33])";
    PrologQueryProxy qr = pl_.query(q.str());
    PrologQueryProxy::iterator it = qr.begin();
    if (it != qr.end())
    {
      PrologBindings te = *it;
      pose.header.frame_id = "map";
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = te["M03"];
      pose.pose.position.y = te["M13"];
      pose.pose.position.z = te["M23"];
      tf::Quaternion quat;
      tf::Matrix3x3 m = tf::Matrix3x3(te["M00"],te["M01"],te["M02"],te["M10"],te["M11"],te["M12"],te["M20"],te["M21"],te["M22"]);
      m.getRotation(quat); 
      pose.pose.orientation.x = quat.x();
      pose.pose.orientation.y = quat.y();
      pose.pose.orientation.z = quat.z();
      pose.pose.orientation.w = quat.w();
      return true;
    }
    else {
      stringstream f;
      f << "Position of " << objInst_ << " is not recorded";
      feedback_.feedback = f.str();
      as_.publishFeedback(feedback_);
      return false;
    }
  }

  /**
   * querys the content of an object from the knowrob database
   * and returns its value 
   *
   * @param objInst  Name of the object instance in knowrob
   * @param content  Content recorded in the database
   */ 
  bool getContentFromDatabase(string objInst, string content)
  {
    stringstream q;
    q << "owl_has(knowrob:'" << objInst << "', knowrob:contains, Cont)";
    PrologQueryProxy qr = pl_.query(q.str());
    PrologQueryProxy::iterator it = qr.begin();
    if (it != qr.end())
    {
      PrologBindings te = *it;
      content = te["Cont"].toString();
      return true;
    }
    else {
      return false;
    }
  }

  /**
   * calls the GetDrinkColor action with the given pose 
   *
   * @param pose     Pose around which to identify color 
   * @param color    Dominant Color 
   */ 
  bool callGetDrinkColorAction(geometry_msgs::PoseStamped pose, string color)
  {
    actionlib::SimpleActionClient<perception_ar_kinect::GetDrinkColorAction> ac ("get_drink_color", true);
    ac.waitForServer();
    perception_ar_kinect::GetDrinkColorGoal goal;
    goal.pose = pose;
    //assume container is a DrinkingGlass
    goal.bb_height = 0.12;
    goal.bb_width = 0.1;
    ac.sendGoal(goal); 
    bool succeeded = ac.waitForResult(ros::Duration(1.0));
    if (succeeded)
    {
      perception_ar_kinect::GetDrinkColorResult result;
      result = *ac.getResult();
      color = result.color;
      std::cout << result.color << std::endl;
      return true;
    }
    else {
      return false;
    }
  }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "query_content");

  QueryContentAction contentAction(ros::this_node::getName());
  ROS_INFO("[query_content_server]Ready for content queries.");
  ros::spin();

  return 0;
} 


