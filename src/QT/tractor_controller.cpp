#include "tractor_controller.h"

TractorController::TractorController(ros::NodeHandle *nh){
	heading_sub = nh->subscribe("heading", 10, &TractorController::headingCB, this);
	position_sub = nh->subscribe("gps/fix", 10, &TractorController::positionCB, this);
	setHeading(42.4);
}

void TractorController::headingCB(const gravl::Hemisphere &msg){
	setHeading(msg.direction);
}

void TractorController::positionCB(const sensor_msgs::NavSatFix &msg){
	QGeoCoordinate pos(msg.latitude, msg.longitude);
	setPosition(pos);
}

void TractorController::setHeading(const float &f){
	if(current_heading == f){
		return;
	}
	current_heading = f;
	emit headingChanged();
}

void TractorController::setPosition(const QGeoCoordinate &c){
	if(current_position == c){
        return;
	}
    current_position = c;
	emit positionChanged();
}

float TractorController::heading() const{
	return current_heading;
}

QGeoCoordinate TractorController::position() const{
	return current_position;
}