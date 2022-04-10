#include "zed_ros_node.h"

ZED_ROS_Node::ZED_ROS_Node():it_(nh_){
  // ros
  pub_img = it_.advertise("zed_image", 1);
  counter = 0;

  // ----> Create Video Capture
  sl_oc::video::VideoParams params;
  params.res = sl_oc::video::RESOLUTION::HD720;
  params.fps = sl_oc::video::FPS::FPS_60;
  sl_oc::video::VideoCapture cap(params);
  if( !cap.initializeVideo() ){
      std::cerr << "Cannot open camera video capture" << std::endl;
      std::cerr << "See verbosity level for more details." << std::endl;
  }
  std::cout << "Connected to camera sn: " << cap.getSerialNumber() << std::endl;
  // <---- Create Video Capture


  #ifdef TEST_FPS
  // Timestamp to check FPS
  lastTime = static_cast<double>(getSteadyTimestamp())/1e9;
  // Frame timestamp to check FPS
  lastFrameTs = 0;
  #endif

  // Infinite video grabbing loop
  while (ros::ok())
  {
    // Get last available frame
    const sl_oc::video::Frame frame = cap.getLastFrame();

    // ----> If the frame is valid we can display it
    if(frame.data!=nullptr){
      #ifdef TEST_FPS
      if(lastFrameTs!=0){
        // ----> System time
        now = static_cast<double>(getSteadyTimestamp())/1e9;
        elapsed_sec = now - lastTime;
        lastTime = now;
        std::cout << "[System] Frame period: " << elapsed_sec << "sec - Freq: " << 1./elapsed_sec << " Hz" << std::endl;
        // <---- System time

        // ----> Frame time
        double frame_dT = static_cast<double>(frame.timestamp-lastFrameTs)/1e9;
        std::cout << "[Camera] Frame period: " << frame_dT << "sec - Freq: " << 1./frame_dT << " Hz" << std::endl;
        // <---- Frame time
      }
      lastFrameTs = frame.timestamp;
      #endif

      // ----> Conversion from YUV 4:2:2 to BGR for visualization
      frameYUV = cv::Mat( frame.height, frame.width, CV_8UC2, frame.data );
      cv::cvtColor(frameYUV,frameBGR,cv::COLOR_YUV2BGR_YUYV);
      // <---- Conversion from YUV 4:2:2 to BGR for visualization

      header.seq = counter; // user defined counter
      header.stamp = ros::Time::now(); // time
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8 , frameBGR);
      img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
      pub_img.publish(img_msg);
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
