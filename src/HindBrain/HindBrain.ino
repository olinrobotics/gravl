/*
  @file         HindBrain.ino
  Purpose: Provides firmware interface from software to hardware, runs
  realtime control/safety loop

  @author       Connor Novak
  @maintainer   Olin GRAVL
  @email        olingravl@gmail.com
  @version      0.2.0
  @date         2020-02-15
*/

// Library & header includes
#include "HindBrain.h"
#include <Arduino.h>     // Microcontroller standard API
#include <Encoder.h>     // Hitch height encoder library
#include "Estop.h"       // OAK Estop
#include "SoftSwitch.h"  // OAK SoftSwitch
#include "RoboClaw.h"    // Motor controller API
// #include <ros/time.h>    // Timestamp verification

// RoboClaws
auto rc1_serial = &Serial1;
auto rc2_serial = &Serial2;
RoboClaw rc1(rc1_serial, RC_TIMEOUT);
RoboClaw rc2(rc2_serial, RC_TIMEOUT);

// Components
Encoder hitchEncoder(HITCH_ENC_A_PIN, HITCH_ENC_B_PIN);
Estop *eStop;
OAKSoftSwitch *autoLight;

// States
boolean isEStopped = false;
boolean isAuto = false;

// Global variables
unsigned long watchdogTimer;
char usrMsg = '\0';

signed int steerCmd = STEER_CMD_CENTER;
unsigned int velCmd = VEL_CMD_STOP;
unsigned int hitchCmd = H_ACTUATOR_CENTER;

// ROS nodes, publishers, subscribers
ros::NodeHandle nh;
ackermann_msgs::AckermannDriveStamped currDrivePose;
geometry_msgs::Pose currHitchPose;
geometry_msgs::Pose desiredHitchPose;

ros::Subscriber<ackermann_msgs::AckermannDriveStamped> driveSub("/cmd_vel", &ackermannCB);
ros::Subscriber<geometry_msgs::Pose> hitchSub("/cmd_hitch", &hitchCB);
ros::Subscriber<std_msgs::Empty> ping("/safety_clock", &watchdogCB);
ros::Subscriber<std_msgs::String> userInput("/user_input", &userInputCB);

ros::Publisher pubDrive("/curr_drive", &currDrivePose);
ros::Publisher hitchPose("/hitch_pose", &currHitchPose);


void setup() {
  // Initialize estop and auto-light switch
  eStop = new Estop(&nh, ESTOP_SENSE_PIN, ESTOP_DEBOUNCE_TIME);
  eStop->onStop(eStopTractor);
  eStop->offStop(eStartTractor);
  pinMode(ESTOP_RELAY_PIN, OUTPUT);
  digitalWrite(ESTOP_RELAY_PIN, HIGH); // Stop tractor for safety
  autoLight = new OAKSoftSwitch(&nh, "/auto_light", AUTO_LIGHT_PIN);

  // Open serial communication with roboclaw
  rc1.begin(RC_BAUD_RATE);
  rc2.begin(RC_BAUD_RATE);

  // Set up ROS node, subscribers, publishers
  nh.getHardware()->setBaud(SERIAL_BAUD_RATE);
  nh.initNode();
  nh.subscribe(driveSub);
  nh.subscribe(hitchSub);
  nh.subscribe(ping);
  nh.subscribe(userInput);
  nh.advertise(pubDrive);
  nh.advertise(hitchPose);

  // Wait for connection
  while(true) {
    if(nh.connected() && (millis() - watchdogTimer < WATCHDOG_TIMEOUT)) {
      break;
    }
    nh.spinOnce();
    delay(1);
  }

  nh.loginfo("Hindbrain connected, running startup calibration sequence");
  delay(500);
  runStartupSequence();
  watchdogTimer = millis();
}


void loop() {
  // Check for connectivity with mid-brain
  checkSerial(&nh);

  // Update current published pose
  updateCurrDrive();
  updateCurrHitchPose();

  hitchCmd = computeHitchMsg();

  // Send updated motor commands to roboclaws
  if (!isEStopped) {
    updateRoboClaw(velCmd, steerCmd, hitchCmd);
  } else {
    stopRoboClaw(&rc1, &rc2);
  }

  // Update node
  nh.spinOnce();
  delay(1);
}


void ackermannCB(const ackermann_msgs::AckermannDriveStamped &msg) {
  // Save steer and vel cmds to global vars.
  auto t_dur = nh.now().toSec() - msg.header.stamp.toSec();
  if (t_dur < 0.05) {
    steerCmd = steerAckToCmd(msg.drive.steering_angle);
    velCmd = velAckToCmd(msg.drive.speed);
  } else {
    nh.logwarn("ackermannCB: old msg ignored");
  }
  #ifdef DEBUG_ACKERMANNCB
    char m[40];
    snprintf(m, sizeof(m), "ackermannCB: %.2f m/s, %.2f deg, %.2f s",
      msg.drive.speed, msg.drive.steering_angle, t_dur);
    nh.loginfo(m);
  #endif
    
}


void hitchCB(const geometry_msgs::Pose &msg){
  desiredHitchPose.position.z = msg.position.z; // In meters from flat ground
  // TODO: Copy over all desired attributes
}


void watchdogCB(const std_msgs::Empty &msg) {
  // Update watchdog timer on receipt of msg
  watchdogTimer = millis();
}


void userInputCB(const std_msgs::String &msg) {
  // Update input global var with msg only if msg is 1 char.
  if (strlen(msg.data) == 1) {
    usrMsg = *msg.data;
    #ifdef DEBUG_USERINPUTCB
    char m[20];
    snprintf(m, sizeof(m), "userInputCB: %s", msg.data);
    nh.loginfo(m);
    #endif
  }
}


void waitForUserVerification() {
  // Block main code until 'y' is received on /user_input.
  while (true) {
    auto cmd = checkUserInput();
    if (cmd == 'y') {
      break;
    }
    nh.spinOnce();
    delay(1);
  }
}


void runStartupSequence() {
  // Run startup & calibration sequence with appropriate user input.
  //TODO create prompt publishing topic

  // Hold Estop & wait for user
  digitalWrite(ESTOP_RELAY_PIN, HIGH);
  nh.loginfo("START: Remove pins, then publish 'y' to /user_input topic");
  waitForUserVerification();

  // Release Estop & set motor cmds
  nh.loginfo("START: Verification received, homing actuators . . .");
  digitalWrite(ESTOP_RELAY_PIN, LOW);

  delay(2000); // Wait for Roboclaw to boot
  nh.spinOnce(); // Need this update after delay, otherwise no msg pub
  steerCmd = STEER_CMD_CENTER;
  velCmd = VEL_CMD_STOP;
  hitchCmd = H_ACTUATOR_CENTER;
  updateRoboClaw(velCmd, steerCmd, hitchCmd);
  delay(2000); // Wait for motor to reach correct positions
  nh.spinOnce();

  // Hold Estop & wait for user
  digitalWrite(ESTOP_RELAY_PIN, HIGH);
  nh.loginfo("START: Re-install pins, then publish 'y' to /user_input topic");
  waitForUserVerification();

  // Release Estop and finish
  digitalWrite(ESTOP_RELAY_PIN, LOW);
  nh.loginfo("START: Verification received, vehicle ready to run.");
}


int steerAckToCmd(float ack){
  // Calculate and threshold Roboclaw cmd given ackermann velocity msg.
  float cmd;

  // Convert from input message to output command
  if (ack > STEER_MSG_CENTER) {
    cmd = map(ack, STEER_MSG_CENTER, STEER_MSG_LEFT, STEER_CMD_CENTER, STEER_CMD_LEFT);
  } else if (ack < STEER_MSG_CENTER) {
    cmd = map(ack, STEER_MSG_RIGHT, STEER_MSG_CENTER, STEER_CMD_RIGHT, STEER_CMD_CENTER);
  } else {
    cmd = STEER_CMD_CENTER;
  }

  // Threshold cmd for safety
  char msg[60];
  if (cmd > STEER_CMD_MAX) {
    nh.logwarn("ERR: commanded steering angle > STEER_CMD_MAX");
    snprintf(msg, sizeof(msg), "ERR: cmd_steer=%d > steer_max=%i", int(cmd), STEER_CMD_MAX);
    nh.logwarn(msg);
    cmd = STEER_CMD_MAX;
  }
  else if (cmd < STEER_CMD_MIN) {
    snprintf(msg, sizeof(msg), "ERR: cmd_steer=%d < steer_min=%i", int(cmd), STEER_CMD_MIN);
    nh.logwarn(msg);
    cmd = STEER_CMD_MIN;
  }

  return cmd;
}


int velAckToCmd(float ack) {
  // Calculate and threshold Roboclaw cmd given ackermann velocity msg.
  float cmd;

  // Convert from range of input signal to range of output signal
  if (ack > VEL_MSG_STOP) {
    cmd = map(ack, VEL_MSG_STOP, VEL_MSG_FWD, VEL_CMD_STOP, VEL_CMD_FWD);
  } else if (ack < VEL_MSG_STOP) {
    cmd = map(ack, VEL_MSG_REV, VEL_MSG_STOP, VEL_CMD_REV, VEL_CMD_STOP);
  } else {
    cmd = VEL_CMD_STOP;
  }

  // Threshold cmd for safety
  char msg[50];
  if (cmd > VEL_CMD_MAX) {
    snprintf(msg, sizeof(msg), "ERR: cmd_vel=%d > vel_max=%i", int(cmd), VEL_CMD_MAX);
    nh.logwarn(msg);
    cmd = VEL_CMD_MAX;
  }
  else if (cmd < VEL_CMD_MIN) {
    snprintf(msg, sizeof(msg), "ERR: cmd_vel=%d < vel_min=%i", int(cmd), VEL_CMD_MIN);
    nh.logwarn(msg);
    cmd = VEL_CMD_MIN;
  }

  return cmd;
}


void checkSerial(ros::NodeHandle *nh) {
  // Given node, estops if watchdog has timed out
  // https://answers.ros.org/question/124481/rosserial-arduino-how-to-check-on-device-if-in-sync-with-host/

  if(millis() - watchdogTimer >= WATCHDOG_TIMEOUT) {
    if(!isEStopped) {
      nh->logerror("Lost connectivity . . .");
      eStopTractor();
    }
  }
}


char checkUserInput() {
  // Get most recent user-sent input (if any), reset global var if necessary.
  auto outMsg = usrMsg;
  usrMsg = '\0';
  return outMsg;
}


void updateRoboClaw(int vel_cmd, int steer_cmd, int hitch_cmd) {
  // Given velocity, steering, and hitch message, sends vals to RoboClaw

  // Write velocity to RoboClaw
  // TODO: add sensor for motor on or not; this is what actually matters.
  rc1.SpeedAccelDeccelPositionM1(RC1_ADDRESS, 100000, 1000, 0, vel_cmd, 1);
  rc1.SpeedAccelDeccelPositionM2(RC1_ADDRESS, 0, 1000, 0, steer_cmd, 1);

  // Write hitch to RoboClaw
  rc2.SpeedAccelDeccelPositionM2(RC2_ADDRESS, 100000, 1000, 0, hitch_cmd, 1);

  // roslog msgs if debugging
  #ifdef DEBUG_UPDATEROBOCLAW
    char m[56];
    snprintf(m, sizeof(m), "DBG: steerCmd = %d, velCmd = %d, hitchCmd = %d", steer_cmd, vel_cmd, hitch_cmd);
    nh.loginfo(m);
  #endif
}


void stopRoboClaw(RoboClaw *rc1, RoboClaw *rc2) {
  // Given roboclaw to stop, publishes messages such that Roboclaw is safe

  // Send velocity pedal to stop position
  rc1->SpeedAccelDeccelPositionM1(RC1_ADDRESS, 100000, 1000, 0, VEL_CMD_STOP, 0);

  // Stop steering motor
  rc1->SpeedM2(RC1_ADDRESS, 0);

  // Send hitch actuator to stop position
  rc2->SpeedAccelDeccelPositionM2(RC2_ADDRESS, 100000, 1000, 0, H_ACTUATOR_CENTER, 0);
}


void updateCurrDrive() {
  // Read encoder values, convert to ackermann drive, publish
  // TODO Fix mapping for steering

  uint32_t encoder1, encoder2;
  rc1.ReadEncoders(RC1_ADDRESS, encoder1, encoder2);
  currDrivePose.drive.speed = encoder1; //mapPrecise(encoder1, VEL_CMD_REV, VEL_CMD_FWD, VEL_MSG_REV, VEL_MSG_FWD);
  currDrivePose.drive.steering_angle = encoder2; //mapPrecise(encoder2, STEER_CMD_RIGHT, STEER_CMD_LEFT, STEER_MSG_RIGHT, STEER_MSG_LEFT);
  pubDrive.publish(&currDrivePose);
}


void updateCurrHitchPose(){
  // Read encoder value, convert to hitch height in meters, publish
  // TODO: What is the hitch height measured from? Where is 0?
  float encoderValInch;
  float hitchHeight;
  long hitchEncoderValue;

  hitchEncoderValue = hitchEncoder.read();
  encoderValInch = hitchEncoderValue / 1000.0; // Inches
  hitchHeight = encoderValInch * -1.1429 * 0.0254; // Meters TODO: What is the -1.1429?
  currHitchPose.position.z = hitchHeight;
  hitchPose.publish(&currHitchPose);
}


int computeHitchMsg() {
  // Take current and desired hitch position to compute actuator position
  int msg;
  auto desired = desiredHitchPose.position.z;
  auto current = currHitchPose.position.z;
  auto error = desired - current;

  // If hitch height is "good enough," move lever to center
  if (abs(error) < ENC_STOP_THRESHOLD) {
    msg = H_ACTUATOR_CENTER;
  } else {
    if (error > 0) {
      // If hitch is too high, move lever forwards
      msg = H_ACTUATOR_MIN;
    } else {
      // If hitch is too low, move lever backwards
      msg = H_ACTUATOR_MAX;
    }
  }
  return msg;
}


void eStopTractor() {
  // Estop tractor
  isEStopped = true;
  digitalWrite(ESTOP_RELAY_PIN, HIGH);
  stopRoboClaw(&rc1, &rc2);
  nh.logerror("Tractor has E-Stopped");
}


void eStartTractor() {
  // Disactivate isEStopped state
  digitalWrite(ESTOP_RELAY_PIN, LOW);
  isEStopped = false;
  nh.loginfo("MSG: EStop Disactivated");
  runStartupSequence();
}


float mapPrecise(float x, float inMin, float inMax, float outMin, float outMax) {
  // Emulate Arduino map() function, uses floats for precision.
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
