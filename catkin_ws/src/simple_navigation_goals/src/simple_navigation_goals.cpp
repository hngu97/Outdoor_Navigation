#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Path.h>
#include <string>
#include <tf/transform_listener.h>
#include "navsat_conversions.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");
  boost::shared_ptr<nav_msgs::Path const> path = NULL;
  int waitTime = 0;
  double utmX, utmY;
  std::string utm_zone_tmp;
  tf::TransformListener listener;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  while (path == NULL)
  {
    path = ros::topic::waitForMessage<nav_msgs::Path>("/jsPath",ros::Duration(2));
    ROS_INFO("Waiting for nav_msgs/path");
    waitTime++;
    if (waitTime == 5)
    {
      ROS_ERROR("Can't find nav_msgs/path");
      return 0;
    }
  }
  waitTime = 0;
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
    waitTime++;
    if (waitTime == 2)
    {
      ROS_ERROR("move_base action server error");
      return 0;
    }
  }
  try
  {
    listener.waitForTransform("utm", "map", ros::Time::now(), ros::Duration(3.0));
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(0.01).sleep();
  }
  move_base_msgs::MoveBaseGoal goal;
  geometry_msgs::PoseStamped new_goal_utm, new_goal_map;
  NavsatConversions::LLtoUTM(path->poses.back().pose.position.x, path->poses.back().pose.position.y, utmY, utmX, utm_zone_tmp);
  new_goal_utm.header.frame_id = "utm";
  new_goal_utm.pose.position.x = utmX;
  new_goal_utm.pose.position.y = utmY;
  new_goal_utm.pose.position.z = 0;
  new_goal_utm.pose.orientation.x = 0;
  new_goal_utm.pose.orientation.y = 0;
  new_goal_utm.pose.orientation.z = 0;
  new_goal_utm.pose.orientation.w = 1;
  listener.transformPose("map", new_goal_utm, new_goal_map);
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = new_goal_map.pose.position.x;
  goal.target_pose.pose.position.y = new_goal_map.pose.position.y;
  goal.target_pose.pose.position.z = 0;
  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1;


  ROS_INFO("FrameID: %s, Final Goal X: %lf, Final Goal Y: %lf", goal.target_pose.header.frame_id.c_str(), goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Goal reached");
  else
    ROS_INFO("Fail to reach goal");

  return 0;
}
