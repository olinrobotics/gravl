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

signed int steerMsg = STEER_MSG_CENTER;
unsigned int velMsg = VEL_MSG_STOP;
unsigned int hitchMsg = H_ACTUATOR_CENTER ;

// ROS nodes, publishers, subscribers
ros::NodeHandle nh;
ackermann_msgs::AckermannDrive currDrivePose;
geometry_msgs::Pose currHitchPose;
geometry_msgs::Pose desiredHitchPose;

ros::Subscriber<ackermann_msgs::AckermannDrive> driveSub("/cmd_vel", &ackermannCB);
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
  autoLight = new OAKSoftSwitch(&nh, "/auto_light", AUTO_LIGHT_PIN);

  // Stop engine for safety
  stopEngine();

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

  hitchMsg = computeHitchMsg();

  // Send updated motor commands to roboclaws
  if (!isEStopped) {
    updateRoboClaw(velMsg, steerMsg, hitchMsg);
  } else {
    stopRoboClaw(&rc1, &rc2);
  }

  // Update node
  nh.spinOnce();
  delay(1);
}


void ackermannCB(const ackermann_msgs::AckermannDrive &msg) {
  // Save steer and vel cmds to global vars.
  steerMsg = steerAckToCmd(msg.steering_angle);
  velMsg = velAckToCmd(msg.speed);
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
    #ifdef DEBUG
    char j[20];
    snprintf(j, sizeof(j), "Received %s", msg.data);
    nh.loginfo(j);
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

  nh.loginfo("Remove pins on steering and velocity, then publish 'y' to /user_input topic");
  waitForUserVerification();

  nh.loginfo("Verification received, homing steering and velocity actuators . . .");
  delay(500);
  rc1.SpeedAccelDeccelPositionM1(RC1_ADDRESS, 0, 300, 0, velMsg, 1);
  nh.loginfo("Flag 1");
  rc1.SpeedAccelDeccelPositionM2(RC1_ADDRESS, 0, 500, 0, steerMsg, 1);
  nh.loginfo("Flag 2");
  rc2.SpeedAccelDeccelPositionM2(RC2_ADDRESS, 0, 300, 0, hitchMsg, 1);
  nh.loginfo("Flag 3");
  delay(500);

  nh.loginfo("Re-install pins on steering and velocity, then publish 'y' to /user_input topic");
  waitForUserVerification();
  nh.loginfo("Verification received, vehicle ready to run.");
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
  if (cmd > STEER_CMD_MAX) {
    cmd = STEER_CMD_MAX;
    nh.logwarn("ERR: commanded steering angle > STEER_CMD_MAX");
  }
  else if (cmd < STEER_CMD_MIN) {
    cmd = STEER_CMD_MIN;
    nh.logwarn("ERR: commanded steering angle < STEER_CMD_MIN");
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
  if (cmd > VEL_CMD_MAX) {
    cmd = VEL_CMD_MAX;
    nh.logwarn("ERR: commanded velocity > VEL_CMD_MAX");
  }
  else if (cmd < VEL_CMD_MIN) {
    cmd = VEL_CMD_MIN;
    nh.logwarn("ERR: commanded velocity < VEL_CMD_MIN");
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


void updateRoboClaw(int velMsg, int steerMsg, int hitchMsg) {
  // Given velocity, steering, and hitch message, sends vals to RoboClaw

  // Write velocity to RoboClaw
  rc1.SpeedAccelDeccelPositionM1(RC1_ADDRESS, 100000, 1000, 0, velMsg, 1);

  // Write steering to RoboClaw if tractor is moving, else returns debug msg
  // TODO: add sensor for motor on or not; this is what actually matters.
  if (velMsg < VEL_CMD_REV - 100) {
    rc1.SpeedAccelDeccelPositionM2(RC1_ADDRESS, 0, 1000, 0, steerMsg, 1);
  } else {
    nh.logwarn("Tractor not moving, steering message rejected");
  }

  // Write hitch to RoboClaw
  rc2.SpeedAccelDeccelPositionM2(RC2_ADDRESS, 100000, 1000, 0, hitchMsg, 1);

  // roslog msgs if debugging
  #ifdef DEBUG
    char j[56];
    snprintf(j, sizeof(j), "DBG: steerMsg = %d, velMsg = %d, hitchMsg = %d", steerMsg, velMsg, hitchMsg);
    nh.loginfo(j);
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
  currDrivePose.speed = mapPrecise(encoder1, VEL_CMD_REV, VEL_CMD_FWD, VEL_MSG_REV, VEL_MSG_FWD);
  currDrivePose.steering_angle = mapPrecise(encoder2, STEER_CMD_RIGHT, STEER_CMD_LEFT, STEER_MSG_RIGHT, STEER_MSG_LEFT);
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


void stopEngine() {
  // Toggle engine stop relay

  digitalWrite(ESTOP_RELAY_PIN, HIGH);
  delay(2000);
  digitalWrite(ESTOP_RELAY_PIN, LOW);
}


void eStopTractor() {
  // Estop tractor
  isEStopped = true;

  nh.logerror("Tractor has E-Stopped");
  stopRoboClaw(&rc1, &rc2);
  stopEngine();
}


void eStartTractor() {
  // Disactivate isEStopped state
  isEStopped = false;

  // Logs verification msg
  char i[24];
  snprintf(i, sizeof(i), "MSG: EStop Disactivated");
  nh.loginfo(i);
}


float mapPrecise(float x, float inMin, float inMax, float outMin, float outMax) {
  // Emulate Arduino map() function, uses floats for precision.
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
