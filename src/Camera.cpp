#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tractor/tractor_cfgConfig.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

cv::Mat threshold;
volatile int low, high;
volatile bool filter;

void FLIRCallback(const sensor_msgs::ImageConstPtr&  msg){
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
	cv::inRange(cv_ptr->image, low, high, threshold);
	if(filter){
		cv::imshow("flir", threshold);
	}
	else{
		cv::imshow("flir", cv_ptr->image);
	}
	cv::waitKey(1);
}

void dynamicCallback(tractor::tractor_cfgConfig &config, uint32_t level){
	low = config.low_filter;
	high = config.high_filter;
	filter = config.filter;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "flir");
	ros::NodeHandle n;
	ros::Subscriber flir = n.subscribe("/camera/usb_cam1/image_raw", 10, FLIRCallback);
	dynamic_reconfigure::Server<tractor::tractor_cfgConfig> server;
	dynamic_reconfigure::Server<tractor::tractor_cfgConfig>::CallbackType f;
	f = boost::bind(&dynamicCallback, _1, _2);
  	server.setCallback(f);
	std::cout << "OpenCV version : " << CV_VERSION << std::endl;
	cv::namedWindow("flir");	
	ros::spin();
    return 0;
}