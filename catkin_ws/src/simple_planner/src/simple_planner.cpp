#include <pluginlib/class_list_macros.h>
#include "simple_planner/simple_planner.h"
#include "simple_planner/navsat_conversions.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(simple_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

//Default Constructor
namespace simple_planner
{

  GlobalPlanner::GlobalPlanner()
  {
    initialized_ = false;
    initialize();
  }

  GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    initialized_ = false;
    initialize(name, costmap_ros);
  }

  void GlobalPlanner::initialize()
  {
    if(!initialized_)
    {
      // initialize other planner parameters
      ROS_INFO("Initialize Simple Planner");
      initialized_ = true;
      utmX = 0;
      utmY = 0;
      goalIndex = 0;
      prev_goal.pose.position.x = 0;
      prev_goal.pose.position.y = 0;
    }
    else
    {
      ROS_INFO("This planner has already been initialized... doing nothing");
    }
  }

  void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    if(!initialized_)
    {
      // initialize other planner parameters
      ROS_INFO("Initialize Simple Planner");
      initialized_ = true;
      utmX = 0;
      utmY = 0;
      goalIndex = 0;
      prev_goal.pose.position.x = 0;
      prev_goal.pose.position.y = 0;
    }
    else
    {
      ROS_INFO("This planner has already been initialized... doing nothing");
    }
  }

  GlobalPlanner::~GlobalPlanner()
  {
  }

  bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan )
  {
    // Initialize
    if(!initialized_)
    {
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }
    // Clear Plan
    plan.clear();
    if ((goal.pose.position.x != prev_goal.pose.position.x) || (goal.pose.position.y != prev_goal.pose.position.y))
    {
      goalIndex = 0;
      goalVector.clear();
      ROS_INFO("Initialize Goals");
      do
      {
        path_ = ros::topic::waitForMessage<nav_msgs::Path>("/jsPath",ros::Duration(2.0));
        ROS_INFO("Wait for Path");
      }
      while(path_ == NULL);
      try
      {
        listener.waitForTransform("utm", "map", ros::Time::now(), ros::Duration(3.0));
      }
      catch (tf::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
        ros::Duration(0.01).sleep();
        //return;
      }
      for (int i = 0; i < path_->poses.size(); i++)
      {
        NavsatConversions::LLtoUTM(path_->poses[i].pose.position.x, path_->poses[i].pose.position.y, utmY, utmX, utm_zone_tmp);
        geometry_msgs::PoseStamped new_goal_utm, new_goal_map;
        new_goal_utm.header.frame_id = "utm";
        new_goal_utm.pose.position.x = utmX;
        new_goal_utm.pose.position.y = utmY;
        new_goal_utm.pose.position.z = 0;
        new_goal_utm.pose.orientation.x = 0;
        new_goal_utm.pose.orientation.y = 0;
        new_goal_utm.pose.orientation.z = 0;
        new_goal_utm.pose.orientation.w = 1;
        listener.transformPose("map", new_goal_utm, new_goal_map);
        new_goal_map.header.frame_id = "map";
        goalVector.push_back(new_goal_map);
        std::cout << i << ": " << new_goal_map.pose.position.x << ", " << new_goal_map.pose.position.y << std::endl;
        prev_goal = new_goal_map;
      }
      ROS_INFO("Finished Initialize Goals");
    }
    if (goalIndex == goalVector.size() - 1)
    {
      plan.push_back(start);
      plan.push_back(goal);
      //ROS_INFO("Final Goal Sent");
      return true;
    }
    geometry_msgs::PoseStamped new_goal;
    new_goal = goalVector[goalIndex];
    new_goal.header.frame_id = "map";
    double delta_x = new_goal.pose.position.x - start.pose.position.x, delta_y = new_goal.pose.position.y - start.pose.position.y;
    double gYaw = 0, gPitch = 0, gRoll = 0;
    double dis_square = pow(delta_x, 2) + pow(delta_y, 2);
    double tolerance_square = pow(2.0, 2);
    gYaw = atan2(delta_y, delta_x);
    rot_euler.setEulerYPR(gYaw, gPitch, gRoll);
    rot_euler.getRotation(rot_quat);
    new_goal.pose.orientation.x = rot_quat.getX();
    new_goal.pose.orientation.y = rot_quat.getY();
    new_goal.pose.orientation.z = rot_quat.getZ();
    new_goal.pose.orientation.w = rot_quat.getW();
    plan.push_back(start);
    new_goal.header.stamp = ros::Time::now();
    plan.push_back(new_goal);
    if ((goalIndex < (goalVector.size() - 1)) && (dis_square < tolerance_square))
    {
      goalIndex++;
    }
    return true;
  }

  /*bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan )
  {
    plan.push_back(start);
    plan.push_back(goal);
    //ROS_INFO("Done");
    return true;
  }*/
};
