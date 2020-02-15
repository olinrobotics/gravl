/**
  hind_brain.ino
  Purpose: Provides firmware interface from software to hardware, runs
  realtime control/safety loop

  @author Connor Novak
  @email connor.novak@students.olin.edu
  @version 0.1.0 18/10/24
*/

// Header file
#include "hind_brain.h"

// Declare switch & estop
Estop *e;
OAKSoftSwitch *l;

// RoboClaws
auto rc1_serial = &Serial1;
RoboClaw rc1(rc1_serial, RC_TIMEOUT);
auto rc2_serial = &Serial2;
RoboClaw rc2(rc2_serial, RC_TIMEOUT);

// Encoders
Encoder hitchEncoder(HITCH_ENC_A_PIN, HITCH_ENC_B_PIN);

//Wheel Lengths
float distanceR = 0.0;
float distanceL = 0.0;
int prevR = 0;
int prevL = 0;
int revsR = 0;
int revsL = 0;
int offsetR = 0;
int offsetL = 0;

// States
boolean isEStopped = false;
boolean isAuto = false;

// Global Variables
unsigned int velMsg = VEL_CMD_STOP;        // Initialize velocity to 0
signed int steerMsg = STEER_CMD_CENTER;   // Initialize steering to straight
unsigned int hitchMsg = H_ACTUATOR_CENTER; // Start actuator in center

char buf[7];
unsigned long watchdog_timer;


// ROS nodes, publishers, subscribers
ros::NodeHandle nh;
ackermann_msgs::AckermannDrive curr_drive_pose;
geometry_msgs::Pose curr_hitch_pose;
geometry_msgs::Pose desired_hitch_pose;
std_msgs::Float64 curr_wheel_enc_right;
std_msgs::Float64 curr_wheel_enc_left;

ros::Subscriber<ackermann_msgs::AckermannDrive> drive_sub("/cmd_vel", &ackermannCB);
ros::Subscriber<geometry_msgs::Pose> hitch_sub("/cmd_hitch", &hitchCB);
ros::Subscriber<std_msgs::Empty> ping("/safety_clock", &watchdogCB);
ros::Subscriber<std_msgs::Float64> wheelResetRight("/wheel_reset_right", &wheelRRCB);
ros::Subscriber<std_msgs::Float64> wheelResetLeft("/wheel_reset_left", &wheelRLCB);
ros::Publisher pub_drive("/curr_drive", &curr_drive_pose);
ros::Publisher hitch_pose("/hitch_pose", &curr_hitch_pose);
ros::Publisher encoderRight("/wheel_enc_right", &curr_wheel_enc_right);
ros::Publisher encoderLeft("/wheel_enc_left", &curr_wheel_enc_left);



void setup() {

  // Initialize estop and auto-switch
  e = new Estop(&nh, ESTOP_PIN, 1);
  pinMode(ESTOP_PIN, OUTPUT);
  e->onStop(eStop);
  e->offStop(eStart);
  l = new OAKSoftSwitch(&nh, "/auto", AUTO_LED_PIN);

  //Stop engine for safety
  stopEngine();

  //Open serial communication with roboclaw
  rc1.begin(38400);
  rc2.begin(38400);

  // Set up ROS node, subscribers, publishers
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(drive_sub);
  nh.subscribe(hitch_sub);
  nh.subscribe(ping);
  nh.subscribe(wheelResetRight);
  nh.subscribe(wheelResetLeft);
  nh.advertise(pub_drive);
  nh.advertise(hitch_pose);
  nh.advertise(encoderRight);
  nh.advertise(encoderLeft);
  

  // Wait for connection
  while(true){
    if(nh.connected() && (millis() - watchdog_timer < WATCHDOG_TIMEOUT)) {break;}
    nh.spinOnce();
    delay(1);
  }

  // Send message of connectivity
  delay(500);
  nh.loginfo("Hindbrain connected; Setting motor positions to neutral.");
  delay(500);

  // TODO Operator verify that all pins are removed

  // Set actuators to default positions
  rc1.SpeedAccelDeccelPositionM1(RC1_ADDRESS, 0, 300, 0, velMsg, 1);
  rc1.SpeedAccelDeccelPositionM2(RC1_ADDRESS, 0, 500, 0, steerMsg, 1);
  rc2.SpeedAccelDeccelPositionM2(RC2_ADDRESS, 0, 300, 0, hitchMsg, 1);

  // Initialize Wheel Encoder Pins
  for(int i = 0; i <=5; i++){
    pinMode(WHEEL_ENC_R_PINS[i],INPUT);
    pinMode(WHEEL_ENC_L_PINS[i],INPUT);
  }

  //Initialize the Wheel Encoders to 0
  bool readingsR[6];
  bool readingsL[6];
  readingsR[0] = !digitalRead(WHEEL_ENC_R_PINS[0]);
  readingsR[1] = !digitalRead(WHEEL_ENC_R_PINS[1]);
  readingsR[2] = !digitalRead(WHEEL_ENC_R_PINS[2]);
  readingsR[3] = !digitalRead(WHEEL_ENC_R_PINS[3]);
  readingsR[4] = !digitalRead(WHEEL_ENC_R_PINS[4]);
  readingsR[5] = !digitalRead(WHEEL_ENC_R_PINS[5]);
  offsetR = readingsR[5] * pow(2,5);
  for(int i = 4; i >= 0; i--){
    readingsR[i] = readingsR[i+1] ^ readingsR[i];
    offsetR += readingsR[i] * pow(2,i);
  }
  readingsL[0] = !digitalRead(WHEEL_ENC_L_PINS[0]);
  readingsL[1] = !digitalRead(WHEEL_ENC_L_PINS[1]);
  readingsL[2] = !digitalRead(WHEEL_ENC_L_PINS[2]);
  readingsL[3] = !digitalRead(WHEEL_ENC_L_PINS[3]);
  readingsL[4] = !digitalRead(WHEEL_ENC_L_PINS[4]);
  readingsL[5] = !digitalRead(WHEEL_ENC_L_PINS[5]);
  offsetL = readingsL[5] * pow(2,5);
  for(int i = 4; i >= 0; i--){
    readingsL[i] = readingsL[i+1] ^ readingsL[i];
    offsetL += readingsL[i] * pow(2,i);
  }
    
  // TODO: make wait until motors reach default positions

  watchdog_timer = millis();
} // setup()

void loop() {

  // Checks for connectivity with mid-brain
  checkSerial(&nh);

  // Update current published pose
  updateCurrDrive();
  updateCurrHitchPose();
  updateWheelPose();

  hitchMsg = computeHitchMsg();

  // Send updated motor commands to roboclaws
  if (!isEStopped) {
    updateRoboClaw(velMsg, steerMsg, hitchMsg);
  } else {
    stopRoboClaw(&rc1, &rc2);
  }

  // Updates node
  nh.spinOnce();
  delay(1);

} // loop()

void ackermannCB(const ackermann_msgs::AckermannDrive &drive) {
  // Callback for ackermann messages; saves data to global vars
  steerMsg = steerAckToCmd(drive.steering_angle);
  velMsg = velAckToCmd(drive.speed);

} // ackermannCB()

void hitchCB(const geometry_msgs::Pose &hitch){
  desired_hitch_pose.position.z = hitch.position.z; // In meters from flat ground
  // TODO: Copy over all desired attributes
} //hitchCB()

void watchdogCB(const std_msgs::Empty &msg) {
  watchdog_timer = millis();
} // watchdogCB()

void wheelRRCB(const std_msgs::Float64 &msg) {
  revsR = int(msg.data / (0.8182 * 3.14));
  offsetR = prevR - (int(64.0 * msg.data/ (0.8182 * 3.14)) % 64);
}

void wheelRLCB(const std_msgs::Float64 &msg) {
  revsL = int(msg.data / (0.8182 * 3.14));
  offsetL = prevL - (int(64.0 * msg.data/ (0.8182 * 3.14)) % 64);
}

void checkSerial(ros::NodeHandle *nh) {
  // Given node, estops if watchdog has timed out
  // https://answers.ros.org/question/124481/rosserial-arduino-how-to-check-on-device-if-in-sync-with-host/

  if(millis() - watchdog_timer >= WATCHDOG_TIMEOUT) {
    if(!isEStopped) {
      nh->logerror("Lost connectivity . . .");
      eStop();
    }
  }
} // checkSerial()

void updateRoboClaw(int velMsg, int steerMsg, int hitchMsg) {
  // Given velocity, steering, and hitch message, sends vals to RoboClaw

  // Write velocity to RoboClaw
  rc1.SpeedAccelDeccelPositionM1(RC1_ADDRESS, 100000, 1000, 0, velMsg, 1);

  // Write steering to RoboClaw if tractor is moving, else returns debug msg
  // TODO: add sensor for motor on or not; this is what actually matters.
  if (velMsg < VEL_CMD_MIN - 100) {rc1.SpeedAccelDeccelPositionM2(RC1_ADDRESS, 0, 1000, 0, steerMsg, 1);}
  else {nh.logwarn("Tractor not moving, steering message rejected");}

  // Write hitch to RoboClaw
  rc2.SpeedAccelDeccelPositionM2(RC2_ADDRESS, 100000, 1000, 0, hitchMsg, 1);

  // roslog msgs if debugging
  #ifdef DEBUG
    char j[56];
    snprintf(j, sizeof(j), "DBG: steerMsg = %d, velMsg = %d, hitchMsg = %d", steerMsg, velMsg, hitchMsg);
    nh.loginfo(j);
  #endif //DEBUG

} // updateRoboClaw()

void stopRoboClaw(RoboClaw *rc1, RoboClaw *rc2) {
  // Given roboclaw to stop, publishes messages such that Roboclaw is safe

  // Send velocity pedal to stop position
  rc1->SpeedAccelDeccelPositionM1(RC1_ADDRESS, 100000, 1000, 0, VEL_CMD_STOP, 0);

  // Stop steering motor
  rc1->SpeedM2(RC1_ADDRESS, 0);

  // Send hitch actuator to stop position
  rc2->SpeedAccelDeccelPositionM2(RC2_ADDRESS, 100000, 1000, 0, H_ACTUATOR_CENTER, 0);
} // stopRoboClaw

void updateCurrDrive() {
  // Read encoder values, convert to ackermann drive, publish
  // TODO Fix mapping for steering

  uint32_t encoder1, encoder2;
  rc1.ReadEncoders(RC1_ADDRESS, encoder1, encoder2);
  curr_drive_pose.speed = mapPrecise(encoder1, VEL_CMD_MIN, VEL_CMD_MAX, VEL_MSG_MIN, VEL_MSG_MAX);
  curr_drive_pose.steering_angle = mapPrecise(encoder2, STEER_CMD_RIGHT, STEER_CMD_LEFT, STEER_MSG_RIGHT, STEER_MSG_LEFT);
  pub_drive.publish(&curr_drive_pose);

} // updateCurrDrive()

void updateCurrHitchPose(){
  // Read encoder value, convert to hitch height in meters, publish
  // TODO: What is the hitch height measured from? Where is 0?
  long hitchEncoderValue;
  float hitchHeight;
  float encoderValInch;
  hitchEncoderValue = hitchEncoder.read();
  encoderValInch = hitchEncoderValue / 1000.0; // Inches
  hitchHeight = encoderValInch * -1.1429 * 0.0254; // Meters TODO: What is the -1.1429?
  curr_hitch_pose.position.z = hitchHeight;
  hitch_pose.publish(&curr_hitch_pose);
} // updateCurrHitchPose()

void updateWheelPose(){
  bool readingsR[6];
  bool readingsL[6];
  int totalR;
  int totalL;
  readingsR[0] = !digitalRead(WHEEL_ENC_R_PINS[0]);
  readingsR[1] = !digitalRead(WHEEL_ENC_R_PINS[1]);
  readingsR[2] = !digitalRead(WHEEL_ENC_R_PINS[2]);
  readingsR[3] = !digitalRead(WHEEL_ENC_R_PINS[3]);
  readingsR[4] = !digitalRead(WHEEL_ENC_R_PINS[4]);
  readingsR[5] = !digitalRead(WHEEL_ENC_R_PINS[5]);
  totalR = readingsR[5] * pow(2,5);
  for(int i = 4; i >= 0; i--){
    readingsR[i] = readingsR[i+1] ^ readingsR[i];
    totalR += readingsR[i] * pow(2,i);
  }
  readingsL[0] = !digitalRead(WHEEL_ENC_L_PINS[0]);
  readingsL[1] = !digitalRead(WHEEL_ENC_L_PINS[1]);
  readingsL[2] = !digitalRead(WHEEL_ENC_L_PINS[2]);
  readingsL[3] = !digitalRead(WHEEL_ENC_L_PINS[3]);
  readingsL[4] = !digitalRead(WHEEL_ENC_L_PINS[4]);
  readingsL[5] = !digitalRead(WHEEL_ENC_L_PINS[5]);
  totalL = readingsL[5] * pow(2,5);
  for(int i = 4; i >= 0; i--){
    readingsL[i] = readingsL[i+1] ^ readingsL[i];
    totalL += readingsL[i] * pow(2,i);
  }
  if(totalR > 48 && prevR < 16){
    revsR--;
  }
  if(totalR < 16 && prevR > 48){
    revsR++; 
  }
  if(totalL > 48 && prevL < 16){
    revsL--;
  }
  if(totalL < 16 && prevL > 48){
    revsL++; 
  }
  prevL = totalL;
  prevR = totalR;
  curr_wheel_enc_right.data = (revsR * 64 + totalR - offsetR) * 0.8182 * 3.14 / 64.0; // diameter * pi / hits per rev
  curr_wheel_enc_left.data = (revsL * 64 + totalL - offsetL) * 0.8182 * 3.14 / 64.0 ;
  encoderRight.publish(&curr_wheel_enc_right);
  encoderLeft.publish(&curr_wheel_enc_left);
  
}

int computeHitchMsg(){
  // Take current and desired hitch position to compute actuator position
  float desired = desired_hitch_pose.position.z;
  float current = curr_hitch_pose.position.z;

  float error = desired - current;

  int hitch_msg;

  // If hitch height is "good enough," then move actuator to neutral position
  if (abs(error) < ENC_STOP_THRESHOLD){
    hitch_msg = H_ACTUATOR_CENTER;
  }
  else{
    if (error > 0){ // Hitch is too high
      hitch_msg = H_ACTUATOR_MIN; // Move lever forwards + hitch down
    }
    else { // Hitch is too low
      hitch_msg = H_ACTUATOR_MAX; // Move lever backwards + hitch up
    }
  }
  return hitch_msg;
}

int steerAckToCmd(float ack_steer){
  //  Given ackermann steering message, returns corresponding RoboClaw command

  // Convert from input message to output command
  if (ack_steer > STEER_MSG_CENTER) {ack_steer = map(ack_steer, STEER_MSG_CENTER, STEER_MSG_LEFT, STEER_CMD_CENTER, STEER_CMD_LEFT);}
  else if (ack_steer < STEER_MSG_CENTER) {ack_steer = map(ack_steer, STEER_MSG_RIGHT, STEER_MSG_CENTER, STEER_CMD_RIGHT, STEER_CMD_CENTER);}
  else { ack_steer = STEER_CMD_CENTER;}

  // Safety limits for signal
  if (ack_steer < STEER_CMD_LEFT) {
    ack_steer = STEER_CMD_LEFT;
    nh.logwarn("ERR: oversteering left");
  }
  else if (ack_steer > STEER_CMD_RIGHT) {
    ack_steer = STEER_CMD_RIGHT;
    nh.logwarn("ERR: oversteering right");
  }

  return ack_steer;
} //steerMsgToCmd

int velAckToCmd(float ack_vel){
  // given ackermann velocity, returns corresponding RoboClaw command

  // Convert from range of input signal to range of output signal
  ack_vel = map(ack_vel, VEL_MSG_MIN, VEL_MSG_MAX, VEL_CMD_MIN, VEL_CMD_MAX);

  // Safety limits for signal; feels switched bc high vals = low speed
  if (ack_vel < VEL_CMD_MAX) {ack_vel = VEL_CMD_MAX;}
  else if(ack_vel > VEL_CMD_MIN) {ack_vel = VEL_CMD_MIN;}

  return ack_vel;
} //velMsgToCmd()

void stopEngine() {
  // Toggles engine stop relay

  digitalWrite(ESTOP_PIN, HIGH);
  delay(2000);
  digitalWrite(ESTOP_PIN, LOW);

} // stopEngine()

void eStop() {
  // Estops tractor

  isEStopped = true;
  nh.logerror("Tractor has E-Stopped");
  stopRoboClaw(&rc1, &rc2);
  stopEngine();
} // eStop()

void eStart() {
  // Disactivates isEStopped state

  isEStopped = false;

  // Logs verification msg
  char i[32];
  snprintf(i, sizeof(i), "MSG: EStop Disactivated");
  nh.loginfo(i);

} // eStart()

float mapPrecise(float x, float inMin, float inMax, float outMin, float outMax) {
  // Emulates Arduino map() function, but uses floats for precision
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;

} // mapPrecise()
