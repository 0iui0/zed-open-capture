#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <vector>
#include <string>
#include <math.h>
#include <signal.h>
#include <Eigen/Geometry>

// ZED
#include "videocapture.hpp"
#include "sensorcapture.hpp"
#include "ocv_display.hpp"
#include "calibration.hpp"
#include "stopwatch.hpp"
#include "stereo.hpp"
#include <iomanip>
#include <opencv2/opencv.hpp>

// Ros
#include <ros/ros.h>
#include <ros/timer.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>

// cv_bridge
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Pcl load and ros
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

using namespace ros;
using namespace pcl;
using namespace std;


class ZED_ROS_Node{
public:
  ZED_ROS_Node();
  void image_ros_bridge();
private:
  NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_left, pub_right, pub_depth;
  ros::Publisher pub_pc;
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg; // >> message to be sent
  std_msgs::Header header; // empty header
  uint64_t counter, lastFrameTs, sn, serial_number;
  int w, h;
  double now, elapsed_sec, lastTime, baseline, fx, fy, cx, cy, remap_elapsed, resize_fact, elapsed, num;
  std::string calibration_file;
  cv::Mat frameRAW8, frameBGR, left_raw, left_rect, right_raw, right_rect, left_for_matcher, right_for_matcher, left_disp_half,left_disp,left_disp_float, left_disp_vis, left_depth_map, left_disp_image;
  cv::Mat map_left_x, map_left_y;
  cv::Mat map_right_x, map_right_y;
  cv::Mat cameraMatrix_left, cameraMatrix_right;
  cv::Ptr<cv::StereoSGBM> left_matcher;
  std::stringstream remapElabInfo, stereoElabInfo;
};
