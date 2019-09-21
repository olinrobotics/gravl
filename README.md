# Overview

This ros package contains base code to run GRAVL's autonomous Kubota tractor. <br/>
For current documentation see the [wiki](https://github.com/olinrobotics/Tractor/wiki).

# Build Status
[![Build Status](https://travis-ci.org/olinrobotics/gravl.svg?branch=master)](https://travis-ci.org/olinrobotics/gravl)


# Quick setup

- Install [ROS](http://wiki.ros.org/)
- `cd <your_catkin_ws>/src>`
- Clone this project: `git clone https://github.com/olinrobotics/gravl.git`
- Navigate back to the catkin works pace root: `cd ..`
- Install dependencies: `rosdep install -iry --from-paths src`
- Build the platform: `catkin_make`
- Run various routines outlined in the [wiki](https://github.com/olinrobotics/Tractor/wiki)

# Dependencies
- [State Controller](https://github.com/olinrobotics/state_controller) - For switching between tractor behaviors and states

# Usage
- [Documentation](https://github.com/olinrobotics/gravl/wiki/Kubo:-Overview) - To run the tractor, see instructions here
