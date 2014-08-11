/*
   TODO: brief overview
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
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include "knowrobMapping.h"

using namespace std;
using namespace json_prolog;

class QueryContentAction
{
private:
  ros::NodeHandle nh_;
  tf::TransformListener tflistener_;
  actionlib::SimpleActionServer<perception_ar_kinect::QueryContentAction> as_;
  std::string action_name_;
  perception_ar_kinect::QueryContentResult result_;
  perception_ar_kinect::QueryContentFeedback feedback_;
  string input_;

  ros::Subscriber sub_;
  Prolog pl_;
  bool observe_;

  bool db_content_av_;
  bool color_av_; 
  string db_content_;
  string color_;
  string objInst_;
  geometry_msgs::PoseStamped objPose_;
  

public:
  QueryContentAction(std::string name) :
    as_(nh_,name, boost::bind(&QueryContentAction::executeCB, this, _1), false),
    action_name_(name),
    observe_(false), db_content_av_(false), color_av_(false)
  {
    sub_ = nh_.subscribe("/ar_pose_markers", 1, &QueryContentAction::markerCallback ,this); 
    as_.start();
  } 

  void executeCB(const perception_ar_kinect::QueryContentGoalConstPtr & goal)
  {
    input_ = goal->container;;

    if(!groundInput(ros::Duration(2.0))) {
      feedback_.feedback = "Input could not be grounded to knowrob individual with recorded pose";
      as_.publishFeedback(feedback_);
      as_.setAborted();
    }
    else
    {
      //query recorded content from database 
      if(getContentFromDatabase(objInst_, db_content_)) {
        //CASE NOT TESTED
        db_content_av_ = true; 
        stringstream f;
        f << "Found the following content in database: " << db_content_;
        feedback_.feedback = f.str();
        as_.publishFeedback(feedback_);
      }
      else {
        feedback_.feedback = "No content record in database";
        as_.publishFeedback(feedback_);
      }
      //look for content color in image
      if(callGetDrinkColorAction(objPose_, color_)) {
        color_av_ = true; 
        stringstream f;
        f << "Found the following color in image: " << color_;
        feedback_.feedback = f.str();
        as_.publishFeedback(feedback_);
      }

      //content recorded in database and color observed in image 
      //NOT TESTED
      if(db_content_av_ && color_av_) {
        if(doContentAndColorMatch()) {
          as_.setSucceeded(result_);
        }
        else { //regard content information as expired and delete it from database
          db_content_av_ = false;
          stringstream q;
          q << "rdf_retractall('" << objInst_ << "', knowrob:contains, _)";
          PrologBindings qr = pl_.once(q.str());
        }
      }

      //only color information from image
      if(color_av_ && !db_content_av_) {
        if(0 == color_.compare("none")) {
          result_.content = "empty";
          as_.setSucceeded(result_);
        }
        else if(identifyContent()) {
          db_content_av_ = false;
          color_av_ = false;
          as_.setSucceeded(result_);
        }
        else {
          db_content_av_ = false;
          color_av_ = false;
          as_.setAborted();
        }
      }
      //only record in database
      else if(db_content_av_) {
        feedback_.feedback = "Object is not visible, but content was recorded in the database previously";
        as_.publishFeedback(feedback_);
        result_.content = db_content_; 
        as_.setSucceeded(result_);
      }
      else {
        feedback_.feedback = "Object is neither visible nor is its content recorded in the database";
        as_.publishFeedback(feedback_);
        result_.content = "unknown";
        as_.setSucceeded(result_);
      }
    }
  }


private:

  void normalizePoseStamped(geometry_msgs::PoseStamped& pose)
  {
    tf::Quaternion n = tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
      pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Quaternion nn = n.normalized();
    pose.pose.orientation.x = nn.x();
    pose.pose.orientation.y = nn.y();
    pose.pose.orientation.z = nn.z();
    pose.pose.orientation.w = nn.w();
  }


  /**
   * ARMarker Callback, activated if knowrob class is passed as goal 
   *
   * finds marker of the same class as specified by goal
   * and identifies the corresponding object instance in database
   */
  void markerCallback(const ar_pose::ARMarkers& markers)
  {
    if(!observe_){
      return;
    }
    const std::vector<ar_pose::ARMarker>& objects = markers.markers; 
    for (const ar_pose::ARMarker& obj : objects)
    {
      string type = knowrob_mapping::markerToObjClass(obj.id);
      if(0 == type.compare(input_))
      {
        //transform pose to map frame
        string targetFrame = "map";
        geometry_msgs::PoseStamped inPose;
        geometry_msgs::PoseStamped outPose;
        inPose.pose = obj.pose.pose;
        inPose.header = obj.header;
        normalizePoseStamped(inPose);
        try {
          tflistener_.waitForTransform(targetFrame, inPose.header.frame_id, ros::Time(0), ros::Duration(10.0));
          tflistener_.transformPose(targetFrame, inPose, outPose); 
          if(knowrob_mapping::findObjInst(type, outPose, objInst_)) { 
            objPose_ = outPose;
            stringstream f;
            f << "Found " << objInst_ << " as currently visible Instance of " << input_;
            feedback_.feedback = f.str();
            as_.publishFeedback(feedback_);
            observe_ = false;
          }
          else {}
        } catch (tf::TransformException ex) {
          ROS_ERROR("[query_content_server]%s", ex.what());
        }
      }
    }
  }


  /**
   * checks wether the content specified in the database has the color found in the image
   */
  bool doContentAndColorMatch()
  {
    stringstream q;
    q << "owl_has('" << db_content_ << "', knowrob:mainColorOfObject, knowrob'" << color_ << "')";
    PrologBindings qr = pl_.once(q.str());
    if (qr.begin() != qr.end()) { // content and color match
      feedback_.feedback = "Content recorded in database and observed color match";
      as_.publishFeedback(feedback_);
      result_.content = db_content_; 
      return true;
    }
    else {
      return false;
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
    stringstream q;
    //query for general content of container class
    q << "owl_has('" << objInst_ << "', rdf:type, _ObjClass),"
      << "class_properties(_ObjClass, knowrob:contains, ContClass),"
    //find content subclasses with correct properties
      << "subclass_by_prop_val(ContClass, knowrob:mainColorOfObject, knowrob:'" << color_ << "', Obj)";
    PrologQueryProxy qp = pl_.query(q.str());
    PrologQueryProxy::iterator it = qp.begin();
    int count = 0;
    stringstream f;
    stringstream conts;
    conts << "[";
    for(it = qp.begin(); it != qp.end(); it++)
    {
      PrologBindings qr = *it;
      if (count == 0) {
        f << "In general " << input_ << " contains substances of type " << qr["ContClass"].toString() << ".\n";
        f << "The following substances of type " << qr["ContClass"].toString() << " with " << color_ << " are known:\n";
      }
      else {
        conts << ",";
      }
      f << qr["Obj"].toString() << "\n";
      conts << "'" << qr["Obj"].toString() << "'"; 
      result_.content = qr["Obj"].toString();
      count++;
    }
    conts << "]";
    feedback_.feedback = f.str();
    as_.publishFeedback(feedback_);
    //content can be clearly identified by properties/color only
    if (count == 1) {
      feedback_.feedback = "Content could be identified by properties/color only";
      as_.publishFeedback(feedback_);
      return true;
    } 
    else if (count == 0) {
      //abort
    }
   
    //else find actions which lead to content change
    q.str("");
    q << "findall(PRS, (member(Drink," << conts.str() << "),"
      <<   "find_cause_of_appearance('" << objInst_ << "', knowrob:contains, Drink, PRS)), _PRSS),"
      << "findall(ActInst, (member(Set, _PRSS), member(Pair, Set), pairs_keys_values([Pair],[Action],[ObjActOn]),"
      <<   "object_possibly_available(ObjActOn, ObjInst),"
      <<   "find_latest_action_inst(Action, [ObjInst], ['" << objInst_ << "'], [], ActInst)), _ActionList),"
      << "sort(_ActionList, _SortedActionList), member(ActionInst, _SortedActionList),"
      << "owl_has(ActionInst, knowrob:'objectActedOn', SourceObj)";

    qp = pl_.query(q.str());
    for(it = qp.begin(); it != qp.end(); it++)
    {
      PrologBindings qr = *it;
      result_.content = qr["SourceObj"].toString(); 
      stringstream f;
      f << qr["ActionInst"].toString() << " was observed previously. \n"
        << "Thus content of " << objInst_ << ", is same as content of " << result_.content << ".";
      feedback_.feedback = f.str();
      as_.publishFeedback(feedback_);
      return true;
      //TODO: in case of more than one solution compare timestamps
    }
    if(qp.begin() == qp.end())
    {
      stringstream f;
      f << "No Action observed which leads to desired changes";
      feedback_.feedback = f.str();
      as_.publishFeedback(feedback_);
      result_.content = "unknown";
      return true;
    }

    //Optional TODO: find content based on the availability of sources 
  }

  


  /**
   * Finds knowrob individual corresponding to input and sets its pose
   *
   * case 1: if input is known individual, its pose is queried from the database
   * case 2: if input is a knowrob class, the next visible instance of that class
   *         is used as an individual 
   *
   * @param timeout  specifies how long MarkerCallback is active
   */ 
  bool groundInput(ros::Duration timeout)
  {
    int r = instOrClass(input_, "Container");
    if(r == 0) { //input param is known individual
      feedback_.feedback = "Input is known individual";
      as_.publishFeedback(feedback_);
      objInst_ = "http://ias.cs.tum.edu/kb/knowrob.owl#";
      objInst_ += input_;
      //query pose from database
      if(getPoseOfObject(input_, objPose_)) {
        return true;
      }
      else {
        feedback_.feedback = "No pose information for input in database";
        as_.publishFeedback(feedback_);
        return false;
      }
    }
    else if(r == 1) { //input param is a class
      feedback_.feedback = "Input is a container class -> activate MarkerCallback";
      as_.publishFeedback(feedback_);
      observe_ = true; //activate MarkerCallback
      ros::Time begin = ros::Time::now();
      ros::Duration d(0.0);
      // wait till correct marker was found or certain time passed
      while(observe_ && (d < timeout)) {
        d = ros::Time::now() - begin; }
      if (observe_) {
        observe_ = false;
        stringstream f;
        f << "Object of class " << input_ << " not visible";
        feedback_.feedback = f.str();
        as_.publishFeedback(feedback_);
        return false;
      }
      else {
        return true;
      }
    }
    else {
      feedback_.feedback = "Input is neither a known individual of type container, nor a container class";
      as_.publishFeedback(feedback_);
      return false;
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
  bool getPoseOfObject(string objInst, geometry_msgs::PoseStamped& pose)
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
      f << "Position of " << objInst << " is not recorded";
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
  bool getContentFromDatabase(string& objInst, string& content)
  {
    stringstream q;
    q << "owl_has('" << objInst << "', knowrob:contains, Cont)";
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
  bool callGetDrinkColorAction(geometry_msgs::PoseStamped pose, string& color)
  {
    actionlib::SimpleActionClient<perception_ar_kinect::GetDrinkColorAction> ac ("get_drink_color", true);
    ac.waitForServer();
    perception_ar_kinect::GetDrinkColorGoal goal;
    goal.pose = pose;
    //assume container is a DrinkingGlass
    goal.bb_height = 0.15;
    goal.bb_width = 0.05;
    ac.sendGoal(goal); 
    bool succeeded = ac.waitForResult(ros::Duration(1.0));
    string state = ac.getState().toString();
    if (succeeded && (0 == state.compare("SUCCEEDED")))
    {
      perception_ar_kinect::GetDrinkColorResult result;
      result = *ac.getResult();
      color = result.color;
      ac.cancelGoal();
      return true;
    }
    else {
      ac.cancelGoal();
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


