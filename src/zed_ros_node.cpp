#include "zed_ros_node.h"

ZED_ROS_Node::ZED_ROS_Node():it_(nh_){
  // ros
  pub_left = it_.advertise("left_raw", 1);
  pub_right = it_.advertise("right_raw", 1);
  pub_depth = it_.advertise("left_depth", 1);
  pub_pc = nh_.advertise<sensor_msgs::PointCloud2>("points", 1);
  counter = 0;

  // ----> Create Video Capture
  sl_oc::video::VideoParams params;
  #ifdef EMBEDDED_ARM
      params.res = sl_oc::video::RESOLUTION::VGA;
  #else
      params.res = sl_oc::video::RESOLUTION::HD720;
  #endif
  params.fps = sl_oc::video::FPS::FPS_30;
  sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::INFO;
  params.verbose = verbose;
  sl_oc::video::VideoCapture cap(params);
  if( !cap.initializeVideo() ){
      std::cerr << "Cannot open camera video capture" << std::endl;
      std::cerr << "See verbosity level for more details." << std::endl;
  }
  sn = cap.getSerialNumber();
  std::cout << "Connected to camera sn: " << sn << std::endl;
  // <---- Create Video Capture


  // ----> Retrieve calibration file from Stereolabs server
  // ZED Calibration
  serial_number = sn;
  // Download camera calibration file
  if( !sl_oc::tools::downloadCalibrationFile(serial_number, calibration_file) )
  {
      std::cerr << "Could not load calibration file from Stereolabs servers" << std::endl;
  }
  std::cout << "Calibration file found. Loading..." << std::endl;

  // ----> Frame size
  cap.getFrameSize(w,h);
  // <---- Frame size

  // ----> Initialize calibration
  baseline=0;
  sl_oc::tools::initCalibration(calibration_file, cv::Size(w/2,h), map_left_x, map_left_y, map_right_x, map_right_y,
                                cameraMatrix_left, cameraMatrix_right, &baseline);

  fx = cameraMatrix_left.at<double>(0,0);
  fy = cameraMatrix_left.at<double>(1,1);
  cx = cameraMatrix_left.at<double>(0,2);
  cy = cameraMatrix_left.at<double>(1,2);

  std::cout << " Camera Matrix L: \n" << cameraMatrix_left << std::endl << std::endl;
  std::cout << " Camera Matrix R: \n" << cameraMatrix_right << std::endl << std::endl;


  // ----> Stereo matcher initialization
  sl_oc::tools::StereoSgbmPar stereoPar;

  //Note: you can use the tool 'zed_open_capture_depth_tune_stereo' to tune the parameters and save them to YAML
  if(!stereoPar.load())
  {
      stereoPar.save(); // Save default parameters.
  }

  left_matcher = cv::StereoSGBM::create(stereoPar.minDisparity,stereoPar.numDisparities,stereoPar.blockSize);
  left_matcher->setMinDisparity(stereoPar.minDisparity);
  left_matcher->setNumDisparities(stereoPar.numDisparities);
  left_matcher->setBlockSize(stereoPar.blockSize);
  left_matcher->setP1(stereoPar.P1);
  left_matcher->setP2(stereoPar.P2);
  left_matcher->setDisp12MaxDiff(stereoPar.disp12MaxDiff);
  left_matcher->setMode(stereoPar.mode);
  left_matcher->setPreFilterCap(stereoPar.preFilterCap);
  left_matcher->setUniquenessRatio(stereoPar.uniquenessRatio);
  left_matcher->setSpeckleWindowSize(stereoPar.speckleWindowSize);
  left_matcher->setSpeckleRange(stereoPar.speckleRange);

  stereoPar.print();
  // <---- Stereo matcher initialization


  // Infinite video grabbing loop
  while (ros::ok())
  {
    // Get last available frame
    const sl_oc::video::Frame frame = cap.getLastFrame();

    // ----> If the frame is valid we can display it
    if(frame.data!=nullptr){
      // ----> Conversion from YUV 4:2:2 to BGR for visualization
      frameYUV = cv::Mat( frame.height, frame.width, CV_8UC2, frame.data );
      cv::cvtColor(frameYUV,frameBGR,cv::COLOR_YUV2BGR_YUYV);
      // <---- Conversion from YUV 4:2:2 to BGR for visualization
      // ----> Extract left and right images from side-by-side
      left_raw = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
      right_raw = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));
      // <---- Extract left and right images from side-by-side


      // ----> Apply rectification
      sl_oc::tools::StopWatch remap_clock;
      cv::remap(left_raw, left_rect, map_left_x, map_left_y, cv::INTER_AREA );
      cv::remap(right_raw, right_rect, map_right_x, map_right_y, cv::INTER_AREA );
      remap_elapsed = remap_clock.toc();
      remapElabInfo << "Rectif. processing: " << remap_elapsed << " sec - Freq: " << 1./remap_elapsed;
      // <---- Apply rectification

      // ----> Stereo matching
      sl_oc::tools::StopWatch stereo_clock;
      resize_fact = 1.0;
      left_for_matcher = left_rect; // No data copy
      right_for_matcher = right_rect; // No data copy
      // Apply stereo matching
      left_matcher->compute(left_for_matcher, right_for_matcher,left_disp_half); //left_disp_half->CV_16UC1
      left_disp_half.convertTo(left_disp_image, CV_8U, 255 / (stereoPar.numDisparities*16.)); //for RVIZ, left_disp_image ->CV_8UC1
      left_disp_half.convertTo(left_disp_float,CV_16UC1);
      cv::multiply(left_disp_float,1./16.,left_disp_float); // Last 4 bits of SGBM disparity are decimal

      elapsed = stereo_clock.toc();
      stereoElabInfo << "Stereo processing: " << elapsed << " sec - Freq: " << 1./elapsed;
      // <---- Stereo matching

      // ----> Extract Depth map
      // The DISPARITY MAP can be now transformed in DEPTH MAP using the formula
      // depth = (f * B) / disparity
      // where 'f' is the camera focal, 'B' is the camera baseline, 'disparity' is the pixel disparity
      num = static_cast<double>(fx*baseline);
      cv::divide(num,left_disp_float,left_depth_map);
      // <---- Extract Depth map

      // ----> Create Point Cloud
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      size_t buf_size = static_cast<size_t>(left_depth_map.cols * left_depth_map.rows);
      for(size_t idx=0; idx<buf_size;idx++ ){
        size_t r = idx/left_depth_map.cols;
        size_t c = idx%left_depth_map.cols;
        ushort depth = left_depth_map.at<ushort>(r, c); //left_depth_map is CV_16U
        // std::cout << depth << " ";
        if(!isinf(depth) && depth >0 && depth > stereoPar.minDepth_mm && depth < stereoPar.maxDepth_mm)
        {
            pcl::PointXYZRGB pt;

            ushort ZZ = static_cast<ushort>(depth); // Z
            float Z = static_cast<float>(ZZ);
            float X = (c-cx)*depth/fx; // X
            float Y = (r-cy)*depth/fy; // Y
            pt.x = X / 1000.;
            pt.y = Y / 1000.;
            pt.z = Z / 1000.;

            cv::Vec3b bgrPixel = left_raw.at<cv::Vec3b>(r, c);
            pt.b = bgrPixel[0];
            pt.g = bgrPixel[1];
            pt.r = bgrPixel[2];

            cloud->push_back(pt);
            if(c==640 && r==360){
              std::cout <<"Depth of the central pixel: "<< depth<< " (mm)"<<std::endl;
              // std::cout<<"x="<< pt.x<<",y="<<pt.y<<",z="<<pt.z<<","<<std::endl;
            }

        }
      }









      header.seq = counter; // user defined counter
      header.stamp = ros::Time::now(); // time
      header.frame_id = "zed_mini";

      sensor_msgs::PointCloud2 ros_points;
      toROSMsg(*cloud, ros_points);
      ros_points.header= header;
      pub_pc.publish(ros_points);

      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1 , left_disp_image);
      img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
      pub_depth.publish(img_msg);
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8 , left_raw);
      img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
      pub_left.publish(img_msg);
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8 , right_raw);
      img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
      pub_right.publish(img_msg);
    }
    // <---- If the frame is valid we can display it

    ros::spinOnce();
  }
}


int main(int argc, char **argv)
{
	init(argc, argv, "zed_ros_node");
	ZED_ROS_Node zrn;
	return 0;
}
