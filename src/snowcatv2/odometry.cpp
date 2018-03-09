#include "odometry.h"

Odometry::Odometry(ros::NodeHandle *nh, const uint8_t left_pin_a, const uint8_t left_pin_b,
					const uint8_t right_pin_a, const uint8_t right_pin_b):nh(nh){
	broadcaster->init(*nh);
	t->header.frame_id = "/odom";
	t->child_frame_id = "/base_link";
	left_wheel = new Encoder(left_pin_a, left_pin_b);
	right_wheel = new Encoder(right_pin_a, right_pin_b);
}

void Odometry::odomPub(){

	left_position = left_wheel->read();
	right_position = right_wheel->read();
	left_wheel->write(0);
	right_wheel->write(0);

	t->transform.translation.x = 1.0; 
	t->transform.rotation.x = 0.0;
	t->transform.rotation.y = 0.0; 
	t->transform.rotation.z = 0.0; 
	t->transform.rotation.w = 1.0;  
	t->header.stamp = nh->now();
	broadcaster->sendTransform(*t);
}