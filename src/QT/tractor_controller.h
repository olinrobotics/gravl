#ifndef TRACTOR_CONTROLLER_H
#define TRACTOR_CONTROLLER_H

#include <QObject>
#include <QGeoCoordinate>

#include <ros/ros.h>
#include <gravl/Hemisphere.h>
#include <sensor_msgs/NavSatFix.h>

class TractorController : public QObject{
	Q_OBJECT
	Q_PROPERTY(float heading READ heading WRITE setHeading NOTIFY headingChanged)
	Q_PROPERTY(QGeoCoordinate position READ position WRITE setPosition NOTIFY positionChanged)

public:
	explicit TractorController(ros::NodeHandle *nh);
signals:
	void positionChanged();
	void headingChanged();
private:
	ros::Subscriber heading_sub;
	ros::Subscriber position_sub;
	float current_heading;
	QGeoCoordinate current_position;
	float heading() const;
	QGeoCoordinate position() const;
	void headingCB(const gravl::Hemisphere &msg);
	void positionCB(const sensor_msgs::NavSatFix &msg);
	void setHeading(const float &f);
	void setPosition(const QGeoCoordinate &c);
};

#endif //TRACTOR_CONTROLLER_H