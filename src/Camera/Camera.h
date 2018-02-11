#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <gravl/gravl_cfgConfig.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

void FLIRCallback(const sensor_msgs::ImageConstPtr&  msg);
void dynamicCallback(gravl::gravl_cfgConfig &config, uint32_t level);
