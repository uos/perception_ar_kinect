//position: 
//  x: 0.1710485816
//  y: -0.0524723976851
//  z: 1.10899984837
//orientation: 
//  x: -0.019898388402
//  y: 0.875484219523
//  z: -0.482836884615
//  w: -0.0118588530901

#include <string>

#include <ros/ros.h>
#include <ros/time.h>

#include <actionlib/server/simple_action_server.h>
#include <perception_ar_kinect/GetDrinkColorAction.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


class GetDrinkColorAction 
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<perception_ar_kinect::GetDrinkColorAction> as_;
  std::string action_name_;
  perception_ar_kinect::GetDrinkColorResult result_;  
  geometry_msgs::PoseStamped goal_;

  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  image_transport::Publisher pub_;
  tf::TransformListener tflistener_;
  image_geometry::PinholeCameraModel cam_model_;
  bool executing_;

public:
  GetDrinkColorAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name), it_(nh_) 
  {
    as_.registerGoalCallback(boost::bind(&GetDrinkColorAction::goalCB, this));
    //subscribe to kinect image
    sub_ = it_.subscribeCamera("/kinect/rgb/image_raw", 1, &GetDrinkColorAction::analyseCB, this);
    pub_ = it_.advertise("image_out", 1);
    as_.start();
  }


 //callback for starting a new GetDrinkColorAction
 void goalCB()
 {
   std::cout << "here" << std::endl;
   goal_ = as_.acceptNewGoal()->pose;
   std::cout << "got activated" << std::endl;
 }


 void analyseCB(const sensor_msgs::ImageConstPtr& image_msg,
                const sensor_msgs::CameraInfoConstPtr& info_msg)
 {
   if(!as_.isActive()){
     return;
   }
   std::cout << "is active" << std::endl;

   cv::Mat image;
   cv_bridge::CvImagePtr input_bridge;
   //convert Image message to an OpenCV IplImage
   try {
     input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
     image = input_bridge->image;
   }
   catch (cv_bridge::Exception& ex) {
     ROS_ERROR("[get_drink_color] Fialed to convert image");
     as_.setAborted();
   }

   cam_model_.fromCameraInfo(info_msg);
   std::string frame_id = goal_.header.frame_id;
   std::cout << "goal: " << goal_.header.frame_id << std::endl;
   std::cout << "cameraFrame: " << cam_model_.tfFrame() << std::endl;
   geometry_msgs::PointStamped pointOut;
   geometry_msgs::PointStamped pointIn;
   pointIn.header = goal_.header;
   pointIn.point = goal_.pose.position;
   // transform goal position to the optical frame of the camera
   try {
     ros::Time acquisition_time = info_msg->header.stamp;
     ros::Duration timeout(1.0 / 5);
     tflistener_.waitForTransform(cam_model_.tfFrame(), frame_id, acquisition_time, timeout);
     tflistener_.transformPoint(cam_model_.tfFrame(), pointIn, pointOut);
   }
   catch (tf::TransformException& ex) {
     ROS_WARN("[get_drink_color] TF exception:\n%s", ex.what());
     as_.setAborted();
   }

   cv::Point3d pt_cv(pointOut.point.x, pointOut.point.y, pointOut.point.z);
   cv::Point2d uv_rect = cam_model_.project3dToPixel(pt_cv);
   cv::Point2d uv = cam_model_.unrectifyPoint(uv_rect);

   //draw into image for debugging
   static const int RADIUS = 3;
   cv::circle(image, uv, RADIUS, CV_RGB(255, 0, 0), -1);
   pub_.publish(input_bridge->toImageMsg());

   result_.color = "unknown";
   as_.setSucceeded(result_);
 }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_drink_color");

  GetDrinkColorAction drinkAction(ros::this_node::getName());
  ROS_INFO("Ready to query for drink color inside image.");
  ros::spin();

  return 0;
}
