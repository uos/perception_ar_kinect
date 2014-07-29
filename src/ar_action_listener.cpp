
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <json_prolog/prolog.h>
#include <ros/time.h>

#include <ar_pose/ARMarkers.h>
#include <ar_pose/ARMarker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


using namespace std;
using namespace json_prolog;


struct ObjWithPose{
  string type;
  int id;
  geometry_msgs::PoseStamped pose;
};

class ARFillingActionListener
{
public:
  ARFillingActionListener()
  {
    hasStartTime = false;
    ros::Subscriber sub_ = n_.subscribe("/ar_pose_markers", 100, &ARFillingActionListener::arCallback, this);
    ros::spin();
  }

private:
  ros::NodeHandle n_;
  tf::TransformListener tflistener_;
  ObjWithPose toLoc;
  ObjWithPose objActOn;
  ros::Time startTime;
  bool hasStartTime;

  //TODO: allow for more than a single potential toLoc

  string markerToObjClass(int id)
  {
    switch(id) {
      case 0: return "TetraPak";
      case 1: return "TetraPak";
      case 2: return "DrinkingGlass";
      case 3: return "DrinkingGlass";
      default: return "HumanScaleObject";}
  }
  
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

  string findObjInst(string& type, geometry_msgs::PoseStamped& pose)
  {
    tf::Quaternion q = tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
      pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Pose p;
    p.getBasis().setRotation(q);

    Prolog prolog;
    stringstream a;
    a << "same_object('http://ias.cs.tum.edu/kb/knowrob.owl#" << type << "', [" 
      <<static_cast<double>(p.getBasis().getRow(0).getX())<<","<< static_cast<double>(p.getBasis().getRow(0).getY())<<","
      <<static_cast<double>(p.getBasis().getRow(0).getZ())<<","<< pose.pose.position.x<<","
      <<static_cast<double>(p.getBasis().getRow(1).getX())<<","<< static_cast<double>(p.getBasis().getRow(1).getY())<<","
      <<static_cast<double>(p.getBasis().getRow(1).getZ())<<","<< pose.pose.position.y<<","
      <<static_cast<double>(p.getBasis().getRow(2).getX())<<","<< static_cast<double>(p.getBasis().getRow(2).getY())<<","
      <<static_cast<double>(p.getBasis().getRow(2).getZ())<<","
      <<pose.pose.position.z << ", 0.0, 0.0, 0.0, 1.0], ObjInst)";
    try {
      PrologBindings r = prolog.once(a.str());
      return r["ObjInst"];
    } catch (json_prolog::PrologQueryProxy::QueryError ex) {
      ROS_ERROR("[ar_action_listener]%s", ex.what());
    }
  }
  
  void arCallback(const ar_pose::ARMarkers& markers)
  {
    Prolog pl;
    const std::vector<ar_pose::ARMarker>& objects = markers.markers;
  
    //observe wether filling action takes place
    for(const ar_pose::ARMarker& obj : objects)
    {
      string type = markerToObjClass(obj.id);
      stringstream s;
      s << "owl_subclass_of('http://ias.cs.tum.edu/kb/knowrob.owl#" << type
        << "', 'http://ias.cs.tum.edu/kb/knowrob.owl#FoodVessel')";
      try {
        PrologQueryProxy res = pl.query(s.str());
        PrologQueryProxy::iterator it=res.begin();
        // object is of type FoodVessel?
        if(it != res.end())
        {
          // transform pose to map frame
          string targetFrame = "map";
          geometry_msgs::PoseStamped inPose;
          geometry_msgs::PoseStamped outPose;
          inPose.pose = obj.pose.pose;
          inPose.header = obj.header;
          normalizePoseStamped(inPose);
          try {
            tflistener_.waitForTransform(targetFrame, inPose.header.frame_id, ros::Time(0), ros::Duration(10.0));
            tflistener_.transformPose(targetFrame, inPose, outPose); 
            tf::Quaternion q = tf::Quaternion(outPose.pose.orientation.x, outPose.pose.orientation.y,
              outPose.pose.orientation.z, outPose.pose.orientation.w);
            tf::Pose p;
            double yaw, pitch, roll;
            p.getBasis().setRotation(q);
            p.getBasis().getEulerYPR(yaw, pitch, roll);
            //start time set?
            if (!hasStartTime){
              // has harizontal orientation, potential objActOn 
              if ((-2.0 < pitch) && (pitch < -1.0)){
                // toLoc set?
                if (!toLoc.type.empty()) {
                  // pose above toLoc?
                  if ((toLoc.pose.pose.position.z < outPose.pose.position.z) &&
                      (toLoc.pose.pose.position.z + 0.2 > outPose.pose.position.z)) {
                    startTime = ros::Time::now();  
                    hasStartTime = true;
                    objActOn.type = type;
                    objActOn.id = obj.id;
                    objActOn.pose = outPose;
                  }
                }
              }
              else if ((-0.1 < pitch) && (pitch < 0.1)){
                toLoc.id = obj.id;
                toLoc.type = type;
                toLoc.pose = outPose;
              }
            }
            else {
              if(obj.id == static_cast<unsigned int>(objActOn.id)) {
                // ids match but no longer horizontally oriented 
                if ((-2.0 > pitch) || (pitch > -1.0)){
                  ros::Time endTime = ros::Time::now();
                  ros::Duration d = endTime - startTime; 
                  hasStartTime = false;
                  // filling action took place if actOnObject was in vertical position for >3sec?
                  if (d.toSec() > 5.0) {
                    //TODO create FIllingActionInst
                    cout << "detected filling Process" << endl;
                    string toLocInst = findObjInst(toLoc.type, toLoc.pose); 
                    string objActOnInst = findObjInst(objActOn.type, objActOn.pose);
                    stringstream a;
                    a << "create_action_inst_perception('http://ias.cs.tum.edu/kb/knowrob.owl#FillingProcess',['"
                      << objActOnInst << "'],['" << toLocInst << "'], []," << startTime.toSec() << ","
                      << endTime.toSec() << ", ActionInst)";
                    try {
                      pl.query(a.str());
                    } catch (json_prolog::PrologQueryProxy::QueryError ex) {
                      ROS_ERROR("[ar_action_listener]%s", ex.what());
                    }
                    cout << a.str() << endl;
                  }
                }
              }
            }
          // has vertical orientation, potential object acted on
        } catch (tf::TransformException ex) {
            ROS_ERROR("[ar_action_listener]%s", ex.what());
        }
      }
      } catch (json_prolog::PrologQueryProxy::QueryError ex) {
        ROS_ERROR("[ar_action_listener]%s", ex.what());
      }
    }
  } 
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ar_action_listener");

  ARFillingActionListener fillinglistener;

  return 0;
}

