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
#include <iomanip>
#include <opencv2/opencv.hpp>

// Ros
#include <ros/ros.h>
#include <ros/timer.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/Trigger.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// cv_bridge
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Pcl load and ros
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
// Pcl icp
#include <pcl/registration/icp.h>
// Pcl downsampling
#include <pcl/filters/voxel_grid.h>

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
  image_transport::Publisher pub_img;

  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg; // >> message to be sent
  std_msgs::Header header; // empty header
  uint64_t counter, lastFrameTs;
  double now, elapsed_sec, lastTime;
  cv::Mat frameYUV, frameBGR;
};
