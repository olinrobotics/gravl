#include "odometry.h"

Odometry::Odometry(ros::NodeHandle *nh):nh(nh){
	broadcaster->init(*nh);
}

void Odometry::odom_pub(){
	t->header.frame_id = "/odom";
	t->child_frame_id = "/base_link";
	t->transform.translation.x = 1.0; 
	t->transform.rotation.x = 0.0;
	t->transform.rotation.y = 0.0; 
	t->transform.rotation.z = 0.0; 
	t->transform.rotation.w = 1.0;  
	t->header.stamp = nh->now();
	broadcaster->sendTransform(*t);
}
