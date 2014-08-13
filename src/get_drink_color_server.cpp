
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

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

class GetDrinkColorAction 
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<perception_ar_kinect::GetDrinkColorAction> as_;
  std::string action_name_;
  perception_ar_kinect::GetDrinkColorResult result_;  
  perception_ar_kinect::GetDrinkColorGoalConstPtr goal_;
  geometry_msgs::PoseStamped pose_;

  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  image_transport::Publisher pub_;
  tf::TransformListener tflistener_;
  image_geometry::PinholeCameraModel cam_model_;

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
    goal_ = as_.acceptNewGoal();
    pose_ = goal_->pose;
  }


  void analyseCB(const sensor_msgs::ImageConstPtr& image_msg,
                 const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    if(!as_.isActive()){
      return;
    }
 
    //convert Image message to an OpenCV IplImage
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex) {
      ROS_ERROR("[get_drink_color] Fialed to convert image");
      as_.setAborted();
    }
 
    //transform goal position to the optical frame of the camera
    cam_model_.fromCameraInfo(info_msg);
    std::string frame_id = pose_.header.frame_id;
    geometry_msgs::PointStamped pointOut;
    geometry_msgs::PointStamped pointIn;
    pointIn.header = pose_.header;
    pointIn.point = pose_.pose.position;
    try {
      ros::Time acquisition_time = info_msg->header.stamp;
      ros::Duration timeout(1.0 / 5);
      tflistener_.waitForTransform(cam_model_.tfFrame(), frame_id, acquisition_time, timeout);
      tflistener_.transformPoint(cam_model_.tfFrame(), pointIn, pointOut);
    }
    catch (tf::TransformException& ex) {
      ROS_WARN("[get_drink_color] TF exception:\n%s", ex.what());
      as_.setAborted();
      return;
    }
 
    //slice the image around given Position
    double h = goal_->bb_height/2;
    double w = goal_->bb_height/2;
    cv::Point3d pt_topleft(pointOut.point.x-w, pointOut.point.y-0.5*h, pointOut.point.z);
    cv::Point3d pt_bottomright(pointOut.point.x+w, pointOut.point.y+1.5*h, pointOut.point.z);
    cv::Point2d uv_tl_rect = cam_model_.project3dToPixel(pt_topleft);
    cv::Point2d uv_br_rect = cam_model_.project3dToPixel(pt_bottomright);
    cv::Point2d uv_tl = cam_model_.unrectifyPoint(uv_tl_rect);
    cv::Point2d uv_br = cam_model_.unrectifyPoint(uv_br_rect);
    cv::Mat slice = image(cv::Rect(uv_tl,uv_br)).clone();
    //output image for debugging
    input_bridge->image = slice;
    pub_.publish(input_bridge->toImageMsg());
 
    //convert image to HSV
    cv::Mat hsv_image;
    cv::cvtColor(slice, hsv_image, CV_BGR2HSV);
    
    //filter for color
    if (isRed(hsv_image)) {
      result_.color = "RedColor";
      as_.setSucceeded(result_);
    }
    else if(isYellow(hsv_image)) {
      result_.color = "YellowColor";
      as_.setSucceeded(result_);
    }
    else {
      result_.color = "none";
      as_.setSucceeded(result_);
    }
  }

private:
  bool isRed(cv::Mat& image)
  {
    cv::Mat mask;
    cv::inRange(image, Scalar(0,60,10), Scalar(80,255,120), mask); 
    //count pixels
    int count = 0;
    for(int i = 0; i < mask.rows; i++) {
      for(int j = 0; j < mask.cols; j++) {
        if (mask.at<int>(i,j) > 0)
          count++;
      }
    }
    if (count > 250) {
      return true; }
    else {
      return false; }
  }

  bool isYellow(cv::Mat& image)
  {
    cv::Mat mask;
    cv::inRange(image, Scalar(19,40,180), Scalar(35,255,255), mask); 
    //count pixels
    int count = 0;
    for(int i = 0; i < mask.rows; i++) {
      for(int j = 0; j < mask.cols; j++) {
        if (mask.at<int>(i,j) > 0)
          count++;
      }
    }
    if (count > 210) {
      return true; }
    else {
      return false; }
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
