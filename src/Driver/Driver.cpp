/******************************************************************************
 * Driver for the tractor
 * @file Driver.cpp
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @version     1.0
 *
 * This is meant to provide the transform for the odom frame
 *
 ******************************************************************************/

#include "Driver.h"

Driver::Driver(){

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tractordriver");
  	Driver d;
}