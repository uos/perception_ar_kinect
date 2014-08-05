#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <perception_ar_kinect/QueryContentAction.h>

using namespace perception_ar_kinect;

void doneCB(const actionlib::SimpleClientGoalState& state,
            const QueryContentResultConstPtr& result)
{
  ROS_INFO("Action_finished: %s", state.toString().c_str());
  ROS_INFO("Result: %s", result->content.c_str());
  ros::shutdown();
} 

void activeCB()
{
  ROS_INFO("Goal just went active");
}

void feedbackCB(const QueryContentFeedbackConstPtr& feedback)
{
  ROS_INFO("Feedback: %s", feedback->feedback.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "query_content_client");

  if(argc != 2)
  {
    ROS_INFO("[query_content_client]Usage: query_content_client Container");
    return -1;
  }

  actionlib::SimpleActionClient<perception_ar_kinect::QueryContentAction> ac("query_content", true);
  ac.waitForServer();

  perception_ar_kinect::QueryContentGoal goal;
  goal.container = argv[1]; 
  ac.sendGoal(goal, &doneCB, &activeCB, &feedbackCB);

  ros::spin();

  return 0;
}
