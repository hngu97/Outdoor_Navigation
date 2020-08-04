#include <cv_bridge/cv_bridge.h>    // OpenCV
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/io.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>


class DEPTH_TO_PC2
{
public:
  DEPTH_TO_PC2(ros::NodeHandle nh)
  {
    camInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera/color/camera_info", nh);
    ROS_INFO("Getting cameara intrinsic parameters");
    fx = camInfo->K[0];
    fy = camInfo->K[4];
    cx = camInfo->K[2];
    cy = camInfo->K[5];
    camFrame = camInfo->header.frame_id;
    std::cout << "fx: " << fx << std::endl;
    std::cout << "fy: " << fy << std::endl;
    std::cout << "cx: " << cx << std::endl;
    std::cout << "cy: " << cy << std::endl;
    std::cout << "Camera Frame: " << camFrame << std::endl;
    depthImageSub.subscribe(nh, "camera/aligned_depth_to_color/image_raw", 2);
    classImageSub.subscribe(nh, "fcn_object_segmentation/output", 2);
    syncMsg.reset(new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>(10), depthImageSub, classImageSub));
    syncMsg->registerCallback(boost::bind(&DEPTH_TO_PC2::syncImageCB, this, _1, _2));
    pc2Pub = nh.advertise<sensor_msgs::PointCloud2>("semantic_sidewalk", 2);
    pc2ObPub = nh.advertise<sensor_msgs::PointCloud2>("semantic_obstacle", 2);
  }
  ~DEPTH_TO_PC2()
  {
  }
private:
  double fx = 0, fy = 0, cx = 0, cy = 0;
  std::string camFrame;
  message_filters::Subscriber<sensor_msgs::Image> depthImageSub;
  message_filters::Subscriber<sensor_msgs::Image> classImageSub;
  boost::shared_ptr<sensor_msgs::CameraInfo const> camInfo;
  boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>>> syncMsg;
  ros::Publisher pc2Pub, pc2ObPub;
  tf::TransformListener tflistener;

  void syncImageCB(const sensor_msgs::ImageConstPtr& depthMsg, const sensor_msgs::ImageConstPtr& classMsg)
   {
    cv_bridge::CvImageConstPtr cvDepthPtr, cvClassPtr;
    try
    {
     cvDepthPtr = cv_bridge::toCvShare(depthMsg, "32FC1");
     //cvClassPtr = cv_bridge::toCvShare(classMsg, sensor_msgs::image_encodings::BGR8);
     cvClassPtr = cv_bridge::toCvShare(classMsg, "32SC1");
    }
    catch (cv_bridge::Exception& e)
    {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
    }
    const cv::Mat depthImg = cvDepthPtr->image;
    const cv::Mat classImg = cvClassPtr->image;

    ros::Time timeNow = ros::Time::now();

    int depthRow = depthImg.rows;
    int depthCol = depthImg.cols;
    unsigned long int depthSize = depthRow * depthCol;

    sensor_msgs::PointCloud2 pc2Msg;
    pc2Msg.header.frame_id = camFrame;
    pc2Msg.header.stamp = timeNow;
    pc2Msg.height = 1;
    pc2Msg.width = depthSize;
    pc2Msg.is_dense = true;
    pc2Msg.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier pc2Mod(pc2Msg);
    pc2Mod.setPointCloud2FieldsByString(2, "xyz", "rgb");
    sensor_msgs::PointCloud2Iterator<float> pX(pc2Msg, "x");
    sensor_msgs::PointCloud2Iterator<float> pY(pc2Msg, "y");
    sensor_msgs::PointCloud2Iterator<float> pZ(pc2Msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> pxR(pc2Msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> pxG(pc2Msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> pxB(pc2Msg, "b");

    sensor_msgs::PointCloud2 pc2ObMsg;
    pc2ObMsg.header.frame_id = camFrame;
    pc2ObMsg.header.stamp = timeNow;
    pc2ObMsg.height = 1;
    pc2ObMsg.width = depthSize;
    pc2ObMsg.is_dense = true;
    pc2ObMsg.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier pc2ObMod(pc2ObMsg);
    pc2ObMod.setPointCloud2FieldsByString(2, "xyz", "rgb");
    sensor_msgs::PointCloud2Iterator<float> pXOb(pc2ObMsg, "x");
    sensor_msgs::PointCloud2Iterator<float> pYOb(pc2ObMsg, "y");
    sensor_msgs::PointCloud2Iterator<float> pZOb(pc2ObMsg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> pxROb(pc2ObMsg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> pxGOb(pc2ObMsg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> pxBOb(pc2ObMsg, "b");

    //cv::Vec3b color;
    unsigned long int count = 0, countOb = 0;
    for(int r = 0; r < depthRow; r++)
    {
      for (int c = 0; c < depthCol; c++)
      {
        if (classImg.at<int>(r,c) == 1)
        {
          *pZ = depthImg.at<float>(r,c)/1000;
          *pX = (*pZ * (c - cx)) / fx;
          *pY = (*pZ * (r - cy)) / fy;
          *pxR = 0;
          *pxG = 255;
          *pxB = 0;
          ++pxR;
          ++pxG;
          ++pxB;
          ++pX;
          ++pY;
          ++pZ;
          count ++;
        }
        else
        {
          *pZOb = depthImg.at<float>(r,c)/1000;
          *pXOb = (*pZOb * (c - cx)) / fx;
          *pYOb = (*pZOb * (r - cy)) / fy;
          *pxROb = 255;
          *pxGOb = 0;
          *pxBOb = 0;
          ++pxROb;
          ++pxGOb;
          ++pxBOb;
          ++pXOb;
          ++pYOb;
          ++pZOb;
          countOb++;
        }
      }
    }

    pc2Pub.publish(pc2Msg);
    pc2ObPub.publish(pc2ObMsg);
  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_to_pc2");
  ROS_INFO("depth_to_pc2");
  ros::NodeHandle nh;
  DEPTH_TO_PC2 depth_to_pc2(nh); //Create global object
  ros::spin();
  return 0;
}
