#include <ros/ros.h>
#include <ros/package.h>
#include <utility>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <fstream>
#include <iostream>
#include <math.h>

// Init variables
double y_pos, x_pos, x_vel, x_vel_time, frequency, delay, yaw_offset, magnetic_declination_radians;
bool zero_altitude, broadcast_utm_transform, broadcast_utm_transform_as_parent_frame, publish_filtered_gps, use_odometry_yaw, wait_for_datum;
XmlRpc::XmlRpcValue datum_config;

/*void getParams()
{
  n.getParam("x_vel", x_vel);
  n.getParam("x_vel_time", x_vel_time);
  n.getParam("frequency", frequency);
  n.getParam("delay", delay);
  n.getParam("magnetic_declination_radians", magnetic_declination_radians);
  n.getParam("yaw_offset", yaw_offset);
  n.getParam("zero_altitude", zero_altitude);
  n.getParam("broadcast_utm_transform", broadcast_utm_transform);
  n.getParam("broadcast_utm_transform_as_parent_frame", broadcast_utm_transform_as_parent_frame);
  n.getParam("publish_filteren.getParamd_gps", publish_filtered_gps);
  n.getParam("use_odometry_yaw", use_odometry_yaw);
  n.getParam("wait_for_datum", wait_for_datum);
  n.getParam("datum", datum_config);
}*/
/*
void writeParams(std::string path_to_param_file, double heading_err)
{
  // Open file
  std::ofstream paramsFile (path_to_param_file.c_str());
  // Write to file
  paramsFile << "frequency: " << frequency << std::endl;
  paramsFile << "delay: " << delay << std::endl;
  paramsFile << "magnetic_declination_radians: " << magnetic_declination_radians << std::endl;
  paramsFile << "yaw_offset: " << yaw_offset << std::endl;
  paramsFile << "zero_altitude: " << std::boolalpha << zero_altitude << std::endl;
  paramsFile << "broadcast_utm_transform: " << std::boolalpha << broadcast_utm_transform << std::endl;
  paramsFile << "broadcast_utm_transform_as_parent_frame: " << std::boolalpha << broadcast_utm_transform_as_parent_frame << std::endl;
  paramsFile << "publish_filtered_gps: " << std::boolalpha << publish_filtered_gps << std::endl;
  paramsFile << "use_odometry_yaw: " << std::boolalpha << use_odometry_yaw << std::endl;
  paramsFile << "wait_for_datum: " << std::boolalpha << wait_for_datum << std::endl;
  paramsFile << "datum: " << datum_config << std::endl;
  // Close file
  paramsFile.close();
}

void filtered_odom_CB(const nav_msgs::Odometry odom_msgs)
{
		y_pos = odom_msgs.pose.pose.position.y;
		x_pos = odom_msgs.pose.pose.position.x;
}
*/
int main(int argc, char **argv)
{
  //Initiate node and set hangle
  /*ros::init(argc, argv, "heading_calibration");
  ROS_INFO("Initiated calibration node");

  ros::NodeHandle n;

  // Initialise publishers and subscribers
  ros::Subscriber sub_odom = n.subscribe("odometry/filtered", 100, filtered_odom_CB);
  ros::Publisher pubVel = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",100);

  // Get parameters from parameer server

  n.getParam("x_vel", x_vel);
  n.getParam("x_vel_time", x_vel_time);
  n.getParam("frequency", frequency);
  n.getParam("delay", delay);
  n.getParam("magnetic_declination_radians", magnetic_declination_radians);
  n.getParam("yaw_offset", yaw_offset);
  n.getParam("zero_altitude", zero_altitude);
  n.getParam("broadcast_utm_transform", broadcast_utm_transform);
  n.getParam("broadcast_utm_transform_as_parent_frame", broadcast_utm_transform_as_parent_frame);
  n.getParam("publish_filtered_gps", publish_filtered_gps);
  n.getParam("use_odometry_yaw", use_odometry_yaw);
  n.getParam("wait_for_datum", wait_for_datum);
  n.getParam("datum", datum_config);

  // set publish rate and calculate no. of messages to count
  int pubRate = 10;
  int numVelMsgs = 10 * pubRate;
  ros::Rate rate(pubRate);

  // Create forward velocity commmands and publish
  geometry_msgs::Twist velmsg;
  velmsg.linear.x = x_vel;
  velmsg.angular.z = 0;
  for(int i = 0; i < numVelMsgs; i++)
  {
      pubVel.publish(velmsg);
      rate.sleep();
  }
  ros::Duration(2).sleep(); // Pause for 2 seconds to prevent quick forwards and backwards movement

  // Read y value in filtered odometry and determine correction to heading
  ros::spinOnce();
  double heading_error = atan2(y_pos, x_pos);
  ROS_INFO("Detected heading error of: %.1f Degrees", 180/M_PI*(heading_error));

  //write params file
  std::string path =  ros::package::getPath("p3dx_2dnav") + "/param/robot_localization_sim/navsat_transform_test.yaml";
  ROS_INFO("Writing calibration results to file...");
  //writeParams(path, heading_error);
  ROS_INFO("Wrote to param file: ");
  std::cout << path.c_str() << std::endl;
  // Create backward commmands and publish
  ROS_INFO("Returning to start...");
  velmsg.linear.x = -1 * x_vel;
  velmsg.angular.z = 0;
  for(int i=0; i< numVelMsgs; i++)
  {
      pubVel.publish(velmsg);
      rate.sleep();
  }
  ROS_INFO("Heading Calibration Complete");
  ROS_INFO("Ending Node...");
  ros::shutdown();*/
  return 0;
}
