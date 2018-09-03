/******************************************************************************
 * Motion class for the Tractor
 * @file motion.cpp
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @version     1.0
 *
 * This is meant to encapsulate the motion components of the tractor including
 * the RoboClaw and Ackermann Messages
 *
 * @class Motion motion.h "motion.h"
 ******************************************************************************/

#include "motion.h"

/*
 * Constructor for the class
 *
 * Initializes a subscriber to /cmd_vel
 * Initializes the Roboclaw and homes the actuators
 * @param[in] nh Memory address of the main ros nodehandle
 */
Motion::Motion(ros::NodeHandle *nh) {
  cmd = new ros::Subscriber<ackermann_msgs::AckermannDrive, Motion>("drive", &Motion::ackermannCallback, this);
  nh->subscribe(*cmd);
  roboclaw = new RoboClaw(&MOTION_ROBOCLAW_SERIAL, 10000);
  roboclaw->begin(MOTION_ROBOCLAW_BAUD);
  this->homeActuators();
}


/*
 * Homes the steering and velocity actuators
 */
void Motion::homeActuators() {
  roboclaw->SpeedAccelDeccelPositionM1(MOTION_ROBOCLAW_ADDRESS, 0, 300, 0, VEL_HOME, 0);
  roboclaw->SpeedAccelDeccelPositionM2(MOTION_ROBOCLAW_ADDRESS, 0, 500, 0, STEER_HOME, 0);
}


/*
 * Callback for the ackermann messages
 *
 * Updates the class's vel and steering messages
 * @param[in] nh Memory address of the main ros nodehandle
 */
void Motion::ackermannCallback(const ackermann_msgs::AckermannDrive &drive) {
  vel_msg = drive.speed;
  steer_msg = drive.steering_angle;
}


/*
 * Updates the position of the steering and velocity actuators
 *
 * Reads the current position of the actuators
 * Converts the ackermann messages from deg|m/s to encoder positions
 * Calculates the step size to take
 * Writes the new positions to the roboclaw
 */
void Motion::updateRoboClaw() {
  
  // Get the current position from the RoboClaw
  roboclaw->ReadEncoders(MOTION_ROBOCLAW_ADDRESS, cur_vel_pos, cur_steer_pos);

  // Convert the current message
  this->velConvert();
  this->steerConvert();

  // Update new_vel_pos based on step
  this->stepActuator(&new_vel_pos, cur_vel_pos, VEL_STEP);
  this->stepActuator(&new_steer_pos, cur_steer_pos, STEER_STEP);
  
  // Write velocity to RoboClaw
  roboclaw->SpeedAccelDeccelPositionM1(MOTION_ROBOCLAW_ADDRESS, 100000, 1000, 0, new_vel_pos, 0);

  // Write steering to RoboClaw if tractor is moving, else returns debug msg
  if (new_vel_pos < VEL_HIGH) {
    roboclaw->SpeedAccelDeccelPositionM2(MOTION_ROBOCLAW_ADDRESS, 0, 1000, 0, new_steer_pos, 0);
  }
  else {
    #ifdef DEBUG
      nh.loginfo("ERR: tractor not moving, steering message failed");
    #endif //DEBUG
  }

  // roslog msgs if debugging
  #ifdef DEBUG
    char j[36];
    snprintf(j, sizeof(j), "DBG: new_steer_pos = %d, new_vel_pos = %d", new_steer_pos, new_vel_pos);
    nh.loginfo(j);
  #endif //DEBUG
}


/*
 * Converts steering degrees to an actuator position
 */
void Motion::steerConvert() {

  // Convert from range of input signal to range of output signal, then shift signal
  new_steer_pos = steer_msg * ((STEER_HIGH - STEER_LOW) / STEER_CONTROL_RANGE) + STEER_HOME;

  // Safety limits for signal (double safety, RoboClaw already does this)
  if (new_steer_pos > STEER_HIGH) {
    new_steer_pos = STEER_HIGH;
  }
  else if (new_steer_pos < STEER_LOW) {
    new_steer_pos = STEER_LOW;
  }

  // Switches steering dir
  new_steer_pos = STEER_HIGH - (new_steer_pos - STEER_LOW);
}


/*
 * Converts m/s to an actuator position
 */
void Motion::velConvert() {

  // filter to remove tractor reversal commands (platform wont back up)
  if (vel_msg < 0) {
    vel_msg = 0;
  }

  // Convert from range of input signal to range of output signal
  new_vel_pos = VEL_HOME - vel_msg * ((VEL_HIGH - VEL_LOW) / VEL_CONTROL_RANGE);
}


/*
 * Calculates the step for the actuator to take
 *
 * The Roboclaw has a buffered input for moving the actuators
 * to a given position. This cannot be interrupted with a new
 * position, so the movement is broken up into smaller steps.
 * If the desired movement is larger than a step, set the
 * movement to one step, otherwise return the desired movement.
 * @param[out] new_pos New actuator position
 * @param[in] cur_pos Current actuator position
 * @param[in] step Maximum step size for the actuator to take
 */
void Motion::stepActuator(uint32_t *new_pos, uint32_t cur_pos, int step) {

  // Checks if stepping is necessary (input signal wants to increase by more than the given step size)
  if (abs(cur_pos - *new_pos) > step) {

    // Logs step verification if debugging
    #ifdef DEBUG
      nh.loginfo("DBG: Stepping signal");
    #endif //DEBUG

    if (*new_pos > cur_pos) {
      // If signal increasing, step up
      *new_pos = cur_pos + step;
    }
    else{
      // Else signal decreasing, step down
      *new_pos = cur_pos - step;
    }
  }
  else {
    *new_pos = cur_pos;
  }
}
