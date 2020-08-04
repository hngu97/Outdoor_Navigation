#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>



int main(int argc, char** argv){
  ros::init(argc, argv, "map_odometry");
  ros::NodeHandle node;
  ros::Publisher map_pub = n.advertise<nav_msgs::Odometry>("map/odometry", 50);
  tf::TransformListener listener;
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;
  ros::Rate loop_rate(10);
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  while (node.ok())
  {
    ros::spinOnce();               // check for incoming messages
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/base_link",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    vel_msg.angular = 4.0 * atan2(transform.getOrigin().y(),
                                transform.getOrigin().x());
    vel_msg.linear = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);
    map_pub.publish(odom);
    loop_rate.sleep();
  }
  return 0;
};
