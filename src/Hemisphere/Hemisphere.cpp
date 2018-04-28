/******************************************************************************
 * Hemisphere 
 * @file Hemisphere.cpp
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @version     1.0
 *
 * This takes the true heading from the Hemisphere GPS and publishes it
 *
 ******************************************************************************/


#include "Hemisphere.h"
#include <boost/algorithm/string.hpp>

// Source for serial stuff http://www.tldp.org/HOWTO/text/Serial-Programming-HOWTO

/*
 * Constructor - advertises the heading, connects to the hemisphere
 */
Hemisphere::Hemisphere(){
	heading = n.advertise<gravl::Hemisphere>("heading", 1000);
	std::string serial_port;
	if(!n.getParam("port", serial_port)){
		ROS_ERROR("No serial port set");
		exit(1);
	}
	fd = open(serial_port.c_str(), O_RDWR|O_NOCTTY);
	if(fd<0){
		ROS_ERROR("Could not open port");
		exit(1);
	}
	tcgetattr(fd, &oldtio);
	bzero(&newtio, sizeof(newtio));

	newtio.c_cflag = BAUDRATE|CRTSCTS|CS8|CLOCAL|CREAD; 
	newtio.c_iflag = IGNPAR|ICRNL;
	newtio.c_oflag = 0;
	newtio.c_lflag = ICANON;

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd,TCSANOW,&newtio);
}

/*
 * Publishes the true heading
 */
void Hemisphere::publish(){
	hem.header.stamp = ros::Time::now();
	hem.direction = strtof(parsed.at(2).c_str(), 0);
	heading.publish(hem);
}

/*
 * Reads the serial stream from the Hemisphere for the heading,
 * parses it, and then calls publish
 */
void Hemisphere::run(){
	while(ros::ok()){
		read(fd,buf,255);
		std::string nmea = buf;
		boost::split(parsed, nmea, [](char c){return c == ',';});
		if(parsed.at(0) == "$PASHR"){
			this->publish();
		}
  		ros::spinOnce();
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "hemisphere");
  	Hemisphere h;
  	h.run();
}
