#ifndef HEMISPHERE_H
#define HEMISPHERE_H

#include <termios.h>                                                         |
#include <stdio.h>                                                           |
#include <unistd.h>                                                          |
#include <fcntl.h>                                                           |
#include <sys/signal.h>                                                      |
#include <sys/types.h>  

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include <tractor/Hemisphere.h>

#define BAUDRATE B19200
#define _POSIX_SOURCE 1

class Hemisphere{
public:
	explicit Hemisphere();
	void run();

private:
	void publish();
	ros::NodeHandle n;
	ros::Publisher heading;
	tractor::Hemisphere hem;

	int fd;
	struct termios oldtio, newtio;
	char buf[255];
	std::vector<std::string> parsed;
};

#endif //HEMISPHERE_H