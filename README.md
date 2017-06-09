# GRAVL Tractor

## Arduino Branch

This branch holds the code for the Arduino that is on the tractor

## Libraries
[ROS](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
[AckermannDrive](http://wiki.ros.org/ackermann_msgs)
[Kangaroo](https://www.dimensionengineering.com/software/KangarooArduinoLibrary/html/annotated.html)

## Fixing libraries
If you are getting an error about serial communication definitions in ROS, replace line 44 in ArduinoHardware.h with:

```#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__)```