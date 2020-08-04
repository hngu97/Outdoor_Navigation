# outdoor_navigation
Broken Outdoor Navigation...

An outdoor navigation framework that moves robot from its current position to the final goal using GPS coordinates from Google Map. This package has many limitation and is used mainly to test navigation as a whole. However, this can be useful for people who would like to:
- Use ROSBRIDGE and Javascript for publishing and subscribing.
- Modify global_planner in move_base and send goal to move_base.
- Configuration file for Google Cartorapher, robot_localization, move_base, sensors.
- Semantic segmentation and how to convert them to 3D Point Cloud by using Realsense D435.

The navigation framework: 
<p align="middle">
  <img src="https://github.com/hngu97/outdoor_navigation/blob/master/non_catkin/images/navigation_framework.png" title="Navigation Framework" width="100%" height="100%">
</p>

## ROS_Web_Google
The ROS_Web_Google is used to: 
- Find and send waypoints to robot.
- Track GPS data (used intially to track raw GPS data and Kalman filter output).

To use:
- Register the key to use Google API: https://developers.google.com/maps/documentation/javascript/get-api-key. Then, modify the key in GoogleMap.html
- For publishing and subscribing using Javacript. Firstly, modify the IP in 'url' part in GoogleMapRosInit.js by using your IP address. Secondly, run ```roslaunch rosbridge_server rosbridge_websocket.launch```.
- Add current and destination -> Get path -> Publish Path. The /jsPath will be pulished.

## catkin_ws
Contains some drivers and other files: 
- ddynamic_reconfigure and realsense-ros: ROS interface for realsense D435. Recommend to clone the newest version here: https://github.com/IntelRealSense/realsense-ros. The realsense driver is very unstable by the time I used them.
- nmea_navsat_driver: ROS interface for some ROS GPS. Recommend to clone the newest version here: https://github.com/ros-drivers/nmea_navsat_driver.
- rosaria: ROS interface for Pioneer robots. Recommend to clone the newest version here: https://github.com/amor-ros-pkg/rosaria.
- rosaria_client: Providing nodes to teleop the Pioneer robots. Recommend to clone the newest version here: https://github.com/pengtang/rosaria_client.
- rplidar_ros: ROS interface for RPLidar A2. Recommend to clone the newest version here: https://github.com/Slamtec/rplidar_ros.
- um7: ROS interface for IMU UM7. Recommend to clone the newest version here: https://github.com/ros-drivers/um7. There is some debate about whether the orientation is correct or not. Please check the Issues and PR to update UM7. 
- image_processing: converting depth image (sensor_msgs/Image) with semantic segmentation data (e.g. 0: sidewalk, 1: street, 2: people, etc.) to 3D semantic segmentation point cloud (sensor_msgs/PointCloud2). The semantic segmentation data can be taken from Google Deeplab (https://github.com/tensorflow/models/tree/master/research/deeplab) or JSK (https://jsk-docs.readthedocs.io/projects/jsk_recognition/en/latest/deep_learning_with_image_dataset/overview.html)
- navigation: ROS navigation (https://github.com/ros-planning/navigation). As GPS waypoints are used and taken from Google Map, there was some modification on how move_base called global_planner. The modification was made in move_base.cpp.  
- simple_planner: The custom global_planner that takes GPS waypoints from Google Map and outputs to local_planner. 
- simple_navigation_goals: Action client that sends final goal to move_base.

To run the Pioneer 3-DX and all the sensors (IMU UM7, 2D Lidar Rplidar A2, Adafruit GPS, no camera): ```roslaunch p3dx_2dnav rosaria_sensors.launch```.
To run the realsense d435 camera: ```roslaunch realsense2_camera rs_pointcloud.launch```. This will publish one channel black and white PointCloud, and RGB image.
To run move_base: ```roslaunch p3dx_2dnav p3dx_real.launch```.
If you have strong computer, feel free to fine-tune DWA to improve it performance.

## sidewalk_detection
Please follow this tutorial for semantic segmentation: https://jsk-docs.readthedocs.io/projects/jsk_recognition/en/latest/deep_learning_with_image_dataset/overview.html. 
The jsk_output contains the pretrained model (npz file) for sidewalk detection around Monash Uni. This is trained using image size of 424x240 so it is better to modify the camera image size to match the traning dataset. 

The image_processing in catkin_ws contains the code to convert 2D semantic segmentation to 3D sidewalk and non-sidewalk PointCloud2. The required input is: 
- camera/color/camera_info (for intrinsic paramer, subscribe only once at the beginning).
- camera/aligned_depth_to_color/image_raw (output from realsense with align_depth = true in launch file)
- fcn_object_segmentation/output (output from JSK semantic segmentation).

The output is:
- semantic_sidewalk (PointCloud2 of sidewwalk)
- semantic_obstacle (PointCloud2 of non-sidewalk, or background)

To use:
- Run ```rosrun image_processing depth_to_pc2```.
<p align="middle">
  <img src="https://github.com/hngu97/outdoor_navigation/blob/master/non_catkin/images/googlemap_1.png" title="Google Map" width="60%" height="40%">
  <img src="https://github.com/hngu97/outdoor_navigation/blob/master/non_catkin/images/mnv2_coco_cityscape1.png" title="Pretrained Google Deeplab MobilenetV2" width="45%" height="45%"> 
  <img src="https://github.com/hngu97/outdoor_navigation/blob/master/non_catkin/images/mnv3_large_cityscapes_trainfine.png" title="Pretrained Google Deeplab MobilenetV3" width="45%" height="45%">
  <img src="https://github.com/hngu97/outdoor_navigation/blob/master/non_catkin/images/sidewalk_1.png" title="JSK Segmentation and PointCloud2" width="45%" height="45%"> 
  <img src="https://github.com/hngu97/outdoor_navigation/blob/master/non_catkin/images/sidwalk_2.png" title="JSK Segmentation and PointCloud2" width="45%" height="45%">
</p>

## Localization
You will need the TF tree that connects /UTM to /map frame. This can be done by using robot_localization package: https://github.com/cra-ros-pkg/robot_localization.

