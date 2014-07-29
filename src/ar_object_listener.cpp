
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <json_prolog/prolog.h>


#include <ar_pose/ARMarkers.h>
#include <ar_pose/ARMarker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


using namespace std;
using namespace json_prolog;



class ARListener
{
public:
  ARListener()
  {
    ros::Subscriber sub = n_.subscribe("/ar_pose_markers", 100, &ARListener::arCallback, this);
    ros::spin();
  }

private:
  ros::NodeHandle n_;
  tf::TransformListener tflistener_;

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
  
  void arCallback(const ar_pose::ARMarkers& markers)
  {
    Prolog pl;
    const std::vector<ar_pose::ARMarker>& objects = markers.markers;
  
    for(const ar_pose::ARMarker& obj : objects)
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

        tf::Quaternion q = tf::Quaternion(outPose.pose.orientation.x, outPose.pose.orientation.y, outPose.pose.orientation.z, outPose.pose.orientation.w);
        tf::Pose p;
        p.getBasis().setRotation(q);
        
        // add object perception to knowrob
        string type = markerToObjClass(obj.id);
        stringstream s;
        s << "create_object_perception_with_instance_check('http://ias.cs.tum.edu/kb/knowrob.owl#" << type <<"', ["
          << static_cast<double>(p.getBasis().getRow(0).getX()) << "," << static_cast<double>(p.getBasis().getRow(0).getY()) << ","
          << static_cast<double>(p.getBasis().getRow(0).getZ()) << "," << outPose.pose.position.x << ","
          << static_cast<double>(p.getBasis().getRow(1).getX()) << "," << static_cast<double>(p.getBasis().getRow(1).getY()) << ","
          << static_cast<double>(p.getBasis().getRow(1).getZ()) << "," << outPose.pose.position.y << ","
          << static_cast<double>(p.getBasis().getRow(2).getX()) << "," << static_cast<double>(p.getBasis().getRow(2).getY()) << ","
          << static_cast<double>(p.getBasis().getRow(2).getZ()) << ","
          << outPose.pose.position.z << ", 0.0, 0.0, 0.0, 1.0], ['ARKinectObjectDetection'], ObjInst)";
        try {
          pl.query(s.str());
        } catch(json_prolog::PrologQueryProxy::QueryError ex) {
          ROS_ERROR("[ar_object_listener]%s", ex.what());
        }
      } catch (tf::TransformException ex) {
          ROS_ERROR("[ar_object_listener]%s", ex.what());
      }
    }
  } 
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ar_object_listener");

  ARListener arlistener;

  return 0;
}

