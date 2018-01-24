/******************************************************************************
 * TractorController 
 * @file tractor_controller.cpp
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @version     1.0
 *
 * This takes the heading and gps position and sends that information to the
 * tractor qml to show up on the map
 *
 * This is based on the plane spotter example found here: 
 * https://doc-snapshots.qt.io/qt5-dev/qtlocation-planespotter-example.html
 ******************************************************************************/


#include "tractor_controller.h"

/*
 * DriveState constructor
 */
TractorController::TractorController(ros::NodeHandle *nh){
	heading_sub = nh->subscribe("heading", 10, &TractorController::headingCB, this);
	position_sub = nh->subscribe("gps/fix", 10, &TractorController::positionCB, this);
	setHeading(42.4);
}

/*
 * Callback function for hemisphere subscriber
 */
void TractorController::headingCB(const gravl::Hemisphere &msg){
	setHeading(msg.direction);
}

/*
 * Callback function for gps subscriber
 */
void TractorController::positionCB(const sensor_msgs::NavSatFix &msg){
	QGeoCoordinate pos(msg.latitude, msg.longitude);
	setPosition(pos);
}

/*
 * Function to set the current heading, checks if the heading has changed
 * if it has, it will emit the headingChanged signal
 * @param[in] new_heading new heading
 */
void TractorController::setHeading(const float &new_heading){
	if(current_heading == new_heading){
		return;
	}
	current_heading = new_heading;
	emit headingChanged();
}

/*
 * Function to set the current position, checks if the position has changed
 * if it has, it will emit the positionChanged signal
 * @param[in] new_position new position
 */
void TractorController::setPosition(const QGeoCoordinate &new_position){
	if(current_position == new_position){
        return;
	}
    current_position = new_position;
	emit positionChanged();
}

/*
 * Function to return the heading
 * @return the current heading
 */
float TractorController::heading() const{
	return current_heading;
}

/*
 * Function to return the coordinate
 * @return the current heading
 */
QGeoCoordinate TractorController::position() const{
	return current_position;
}