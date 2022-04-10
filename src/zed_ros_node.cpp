#include "zed_ros_node.h"

ZED_ROS_Node::ZED_ROS_Node():it_(nh_){
  // ros
  pub_left = it_.advertise("left_raw", 1);
  pub_right = it_.advertise("right_raw", 1);
  pub_depth = it_.advertise("left_disp_float", 1);
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
      left_matcher->compute(left_for_matcher, right_for_matcher,left_disp_half);

      left_disp_half.convertTo(left_disp_float,CV_32FC1);
      cv::multiply(left_disp_float,1./16.,left_disp_float); // Last 4 bits of SGBM disparity are decimal
      left_disp = left_disp_float;

      elapsed = stereo_clock.toc();
      stereoElabInfo << "Stereo processing: " << elapsed << " sec - Freq: " << 1./elapsed;
      // <---- Stereo matching





      header.seq = counter; // user defined counter
      header.stamp = ros::Time::now(); // time

      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1 , left_disp_float);
      img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
      pub_depth.publish(img_msg);
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8 , left_raw);
      img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
      pub_left.publish(img_msg);
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8 , right_raw);
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
