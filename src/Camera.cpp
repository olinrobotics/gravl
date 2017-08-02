#include <stdio.h>
#include "Camera.h"

cv::Mat threshold;
volatile int low, high;
volatile bool filter;

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
