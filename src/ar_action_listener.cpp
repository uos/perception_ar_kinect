
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

#include "knowrobMapping.h"

using namespace std;
using namespace json_prolog;


struct ObjWithPose{
  string type;
  int id;
  geometry_msgs::PoseStamped pose;
  double pitch;
};

class ARFillingActionListener
{
public:
  ARFillingActionListener()
  {
    hasStartTime = false;
    ros::Subscriber sub_ = n_.subscribe("/ar_pose_markers", 1, &ARFillingActionListener::arCallback, this);
    while (ros::ok()){
      ros::Duration(0.5).sleep();
      ros::spinOnce();
    }
  }

private:
  ros::NodeHandle n_;
  tf::TransformListener tflistener_;
  ObjWithPose visible[9];
  ObjWithPose toLoc;
  ObjWithPose objActOn;
  ros::Time startTime;
  bool hasStartTime;

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

  
  void arCallback(const ar_pose::ARMarkers& markers)
  {
    Prolog pl;
    const std::vector<ar_pose::ARMarker>& objects = markers.markers;
    int count = 0;  

    //find all markers of type FoodVessel and transform their poses to map frame 
    for(const ar_pose::ARMarker& obj : objects)
    {
      string type = knowrob_mapping::markerToObjClass(obj.id);
      stringstream s;
      // object is of type FoodVessel?
      s << "owl_subclass_of('http://ias.cs.tum.edu/kb/knowrob.owl#" << type
        << "', 'http://ias.cs.tum.edu/kb/knowrob.owl#FoodVessel')";
      try {
        PrologQueryProxy res = pl.query(s.str());
        PrologQueryProxy::iterator it=res.begin();
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
            visible[count].type = type;
            visible[count].id = obj.id;
            visible[count].pose = outPose; 
            visible[count].pitch = pitch;
            count++;
          } catch (tf::TransformException ex) {
              ROS_ERROR("[ar_action_listener]%s", ex.what());
          }
        }
      } catch (json_prolog::PrologQueryProxy::QueryError ex) {
        ROS_ERROR("[ar_action_listener]%s", ex.what());
      }
    }

    //between all visible FoodVessels check for FillingAction
    for(int i = 0; i < count; i++)
    {  
      ObjWithPose source = visible[i];
      if (!hasStartTime){
        // has harizontal orientation, potential Source(objActOn)
        if (((-2.0 < source.pitch) && (source.pitch < -0.4)) || ((2.0 > source.pitch) && (source.pitch > 0.4)))
        {
          geometry_msgs::Point s = source.pose.pose.position;
          //find Target(toLoc)
          for(int j = 0; j < count; j++)
          {
            ObjWithPose target = visible[j];
            geometry_msgs::Point t = target.pose.pose.position;
            //has vertical orientation below Source(objActOn)
            if((-0.1 < target.pitch) && (target.pitch < 0.1) &&
               (t.z < s.z) && (t.z + 0.2 > s.z) &&
               (((t.x-s.x)*(t.x-s.x)+(t.y-s.y)*(t.y-s.y))<0.1))
            {
              objActOn = source;
              toLoc = target;
              startTime = ros::Time::now();  
              hasStartTime = true;
              break;
            }
          }
        }
      }
      else {
        if(source.id == static_cast<unsigned int>(objActOn.id)) {
          // ids match but no longer horizontally oriented 
          if ((-2.0 > source.pitch) || (source.pitch > 2.0) || ((source.pitch > -0.4) && (source.pitch < 0.4)))
          {
            ros::Time endTime = ros::Time::now();
            ros::Duration d = endTime - startTime; 
            hasStartTime = false;
            // filling action took place if actOnObject was in vertical position for >3sec?
            if (d.toSec() > 5.0) {
              ROS_INFO("[ar_action_listener]Detected filling Process");
              string toLocInst;
              string objActOnInst;
              bool map_loc = knowrob_mapping::findObjInst(toLoc.type, toLoc.pose, toLocInst); 
              bool map_acton = knowrob_mapping::findObjInst(objActOn.type, objActOn.pose, objActOnInst);
              if (map_loc && map_acton)
              {
                stringstream a;
                a << "create_action_inst_perception('http://ias.cs.tum.edu/kb/knowrob.owl#FillingProcess',['"
                  << objActOnInst << "'],['" << toLocInst << "'], []," << startTime << ","
                  << endTime << ", ActionInst)";
                try {
                  pl.query(a.str());
                } catch (json_prolog::PrologQueryProxy::QueryError ex) {
                  ROS_ERROR("[ar_action_listener]%s", ex.what());
                }
              }
              else {
                ROS_INFO("[ar_action_listener]Objects involved in detected Action could not be grounded in database");
              }
            }
          }
        }
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

