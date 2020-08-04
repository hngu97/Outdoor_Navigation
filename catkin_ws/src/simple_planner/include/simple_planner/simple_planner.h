#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
//#include <angles/angles.h>
//#include <cstdlib>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
//#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_listener.h>



#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP
namespace simple_planner
{
  class GlobalPlanner: public nav_core::BaseGlobalPlanner
  {
    public:
      GlobalPlanner();
      void initialize();

      GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      ~GlobalPlanner();

      bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
    //protected:
    private:
      bool initialized_;
      double utmX, utmY;
      std::string utm_zone_tmp;
      tf::Matrix3x3 rot_euler;
      tf::Quaternion rot_quat;
      geometry_msgs::PoseStamped prev_goal;
      boost::shared_ptr<nav_msgs::Path const> path_;
      std::vector<geometry_msgs::PoseStamped> goalVector;
      tf::TransformListener listener; //create transformlistener object called listener
      int goalIndex;
      //boost::shared_ptr<sensor_msgs::NavSatFix const> navsatfix_msg_ = NULL;
   };
};
#endif
