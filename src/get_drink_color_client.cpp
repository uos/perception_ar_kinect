#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <perception_ar_kinect/GetDrinkColorAction.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/time.h>



int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_get_drink_color");

  // create the action client
  actionlib::SimpleActionClient<perception_ar_kinect::GetDrinkColorAction> ac("get_drink_color", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(ros::Duration(2.0)); //will wait for infinite time

  // send a goal to the action
  perception_ar_kinect::GetDrinkColorGoal goal;

  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = "/kinect_rgb_optical_frame";
  ps.header.stamp = ros::Time::now();
  ps.pose.position.x = 0.1710485816;
  ps.pose.position.y = -0.0524723976851;
  ps.pose.position.z = 1.10899984837;
  ps.pose.orientation.x = -0.019898388402;
  ps.pose.orientation.y = 0.875484219523;
  ps.pose.orientation.z = -0.482836884615;
  ps.pose.orientation.w = -0.0118588530901;

  goal.pose = ps;
  ac.sendGoal(goal);

  ROS_INFO("Action server started, sending goal.");

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
