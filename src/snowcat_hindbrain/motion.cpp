#include "motion.h"

Motion::Motion(ros::NodeHandle *nh, HardwareSerial *rc, const long baud, const uint8_t address):address(address), lspeed(0), rspeed(0){
	status = new ros::Publisher("/roboclawstatus", &stats);
	cmd = new ros::Subscriber<geometry_msgs::Twist, Motion>("/cmd_vel", &Motion::vel_callback, this);
	nh->advertise(*status);
	nh->subscribe(*cmd);
	roboclaw = new RoboClaw(rc, 10000);
	roboclaw->begin(baud);
	roboclaw->SpeedM1(address, 0);
	roboclaw->SpeedM2(address, 0);
}

void Motion::stat_pub(){
	stats.main_voltage = roboclaw->ReadMainBatteryVoltage(address);
	stats.logic_voltage = roboclaw->ReadLogicBatteryVoltage(address);
	roboclaw->ReadCurrents(address, stats.motor1_current, stats.motor2_current);
	stats.error = roboclaw->ReadError(address);
	status->publish(&stats);
}

void Motion::vel_callback(const geometry_msgs::Twist &vel){
  	//Speed is quad pulses/sec
  	vel.angular.z;
	lspeed = QPPS_PER_REV * REV_PER_METER * vel.linear.x;
	rspeed = QPPS_PER_REV * REV_PER_METER * vel.linear.x;
	roboclaw->SpeedM1(address, 1);
	roboclaw->SpeedM2(address, 1);
}
