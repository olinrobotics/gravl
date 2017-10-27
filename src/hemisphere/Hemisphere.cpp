#include "Hemisphere.h"
#include <boost/algorithm/string.hpp>

// Source for serial stuff http://www.tldp.org/HOWTO/text/Serial-Programming-HOWTO

Hemisphere::Hemisphere(){
	heading = n.advertise<tractor::Hemisphere>("heading", 1000);
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

void Hemisphere::publish(){
	
	
}

void Hemisphere::run(){
	while(true){
		read(fd,buf,255);
		std::string nmea = buf;
		boost::split(parsed, nmea, [](char c){return c == ',';});
		this->publish();
  		ros::spinOnce();
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "hemisphere");
  	Hemisphere h;
  	h.run();
}