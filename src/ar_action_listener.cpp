
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
    ros::Subscriber sub = n_.subscribe("/ar_pose_markers", 100, &ARFillingActionListener::arCallback, this);
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
        try {
          tflistener_.waitForTransform(targetFrame, inPose.header.frame_id, ros::Time(0), ros::Duration(10.0));
          tflistener_.transformPose(targetFrame, inPose, outPose); 
          // check for rotation

          tf::Quaternion q = tf::Quaternion(outPose.pose.orientation.x, outPose.pose.orientation.y,
            outPose.pose.orientation.z, outPose.pose.orientation.w);
          tf::Pose p;
          double yaw, pitch, roll;
          p.getBasis().setRotation(q);
          p.getBasis().getEulerYPR(yaw, pitch, roll);
          //start time set?
          if (!hasStartTime){
            // has harizontal orientation, potential objActOn 
            if ((-1.5 < pitch) && (pitch < -0.8)){
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
            if(obj.id == static_cast<int>(objActOn.id)) {
              // ids match but no longer above
              if ((toLoc.pose.pose.position.z > outPose.pose.position.z) || 
                  (toLoc.pose.pose.position.z + 0.2 < outPose.pose.position.z)) {
                ros::Time endTime = ros::Time::now();
                ros::Duration d = endTime - startTime; 
                hasStartTime = false;
                // filling action took place if actOnObject was in vertical position for >3sec?
                if (d.toSec() > 3.0) {
                  //TODO create FIllingActionInst
                  cout << "found filling Process" << endl;
                  stringstream a;
                  a << " "; 
/*      // add object perception to knowrob
        s << "create_object_perception_with_instance_check('http://ias.cs.tum.edu/kb/knowrob.owl#" << type <<"', ["
          << static_cast<double>(p.getBasis().getRow(0).getX()) << "," << static_cast<double>(p.getBasis().getRow(0).getY()) << ","
          << static_cast<double>(p.getBasis().getRow(0).getZ()) << "," << outPose.pose.position.x << ","
          << static_cast<double>(p.getBasis().getRow(1).getX()) << "," << static_cast<double>(p.getBasis().getRow(1).getY()) << ","
          << static_cast<double>(p.getBasis().getRow(1).getZ()) << "," << outPose.pose.position.y << ","
          << static_cast<double>(p.getBasis().getRow(2).getX()) << "," << static_cast<double>(p.getBasis().getRow(2).getY()) << ","
          << static_cast<double>(p.getBasis().getRow(2).getZ()) << ","
          << outPose.pose.position.z << ", 0.0, 0.0, 0.0, 1.0], ['ARKinectObjectDetection'], ObjInst)";
        pl.query(s.str());
        cout << s.str() << endl;
*/
                }
              }
            }
          }
          // has vertical orientation, potential object acted on
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
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

