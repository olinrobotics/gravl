/**********************************************************************
 * KUBO Hindbrain Code (Teensy 3.5)
 * @file hind_brain.ino
 * @author: Connor Novak
 * @email: connor.novak@students.olin.edu
 * @version: 1.4
 *
 * Basic OAK_compatible control of velocity actuator and
 * steering actuator through ackermann steering messages
 * over /drive, autonomous activation through boolean
 * message over /auto, estop capability over /softestop
 **********************************************************************/

 #include "hind_brain.h"

// Declare switch & estop
Estop *e;
OAKSoftSwitch *l;

// Def/Init Constants

// RoboClaw & Settings
auto rc1_serial = &Serial1;
#define ADDRESS 0x80
#define ROBOCLAW_UPDATE_RATE 500
RoboClaw rc(rc1_serial, 10000);

// General Constants
#define DEBUG TRUE
const int VEL_HIGH = 2048;
const int VEL_LOW = 190;
const int VEL_CONTROL_RANGE = 2;    // Range of incoming signals
const int STEER_HIGH = 1200;
const int STEER_LOW = 600;
const int STEER_CONTROL_RANGE = 90;
const byte VEL_FIDELITY = 10;       // Stepping sub-division of actuator
const byte STEER_FIDELITY = 1;


// Def/Init Global Variables ----------V----------V----------V

// States
boolean isEStopped = false;
boolean isAuto = false;

int prevVelMsg;
unsigned int velMsg = VEL_HIGH;                     // High vel var = low vel
int prevSteerMsg;
signed int steerMsg = (STEER_HIGH + STEER_LOW) / 2; // Straight steer in middle
unsigned long prevMillis = millis();

// function prototypes
void ackermannCB(const ackermann_msgs::AckermannDrive&);
void eStop();
void eStart();

// ROS nodes, publishers, subscribers
ros::NodeHandle nh;
ros::Subscriber<ackermann_msgs::AckermannDrive> sub("drive", &ackermannCB);


void setup() {

  //Open serial communication with roboclaw
  rc.begin(38400);

  // Set up ROS node and initialize subscriber
  nh.getHardware()->setBaud(115200);
  nh.initNode(); // Initialize ROS nodehandle
  nh.subscribe(sub);

  // Initialize estop and auto-switch
  e = new Estop(&nh, ESTOP_PIN, 1);
  pinMode(ESTOP_PIN, OUTPUT);
  l = new OAKSoftSwitch(&nh, "/auto", AUTO_LED_PIN);

  // Provide estop and estart functions
  e->onStop(eStop);
  e->offStop(eStart);

  // TODO Operator verify that actuators are in default positions

  // Set actuators to default positions
  rc.SpeedAccelDeccelPositionM1(ADDRESS, 0, 300, 0, velMsg, 0);
  prevVelMsg = velMsg;
  rc.SpeedAccelDeccelPositionM2(ADDRESS, 0, 500, 0, steerMsg, 0);
  prevSteerMsg = steerMsg;

} // setup()


void loop() {

  // Checks for connectivity with mid-brain and updates estopped state
  checkSerial(&nh);

  // Sends commands to RoboClaw every ROBOCLAW_UPDATE_RATE milliseconds
  if (millis() - prevMillis > ROBOCLAW_UPDATE_RATE && !isEStopped) {
    updateRoboClaw(velMsg, steerMsg);
  }

  // Updates node
  nh.spinOnce();
  delay(1);

} // loop()


void ackermannCB(const ackermann_msgs::AckermannDrive &drive) {
  // Callback for ackermann messages; saves data to global vars
  steerMsg = steerConvert(drive.steering_angle);
  velMsg = velConvert(drive.speed);

}  //ackermannCB()


void checkSerial(ros::NodeHandle *nh) {
  // Given node, estops if node is not connected

  if(!nh->connected()) {
    if(!isEStopped) {eStop();}
  }
}  //checkSerial()


void updateRoboClaw(int velMsg, int steerMsg) {
  // Given velocity and steering message, sends vals to RoboClaw
  // TODO: update to take roboclaw as arg

  //Calculate step sizes based on fidelity
  int steerStep = (STEER_HIGH - STEER_LOW) / STEER_FIDELITY;
  int velStep = (VEL_HIGH - VEL_LOW) / VEL_FIDELITY;

  //Update velMsg based on step
  stepActuator(&velMsg, &prevVelMsg, velStep);
  stepActuator(&steerMsg, &prevSteerMsg, steerStep);

  // Update prev msgs
  prevVelMsg = velMsg;
  prevSteerMsg = steerMsg;

  // Write velocity to RoboClaw
  rc.SpeedAccelDeccelPositionM1(ADDRESS, 100000, 1000, 0, velMsg, 0);

  // Write steering to RoboClaw if tractor is moving, else returns debug msg
  if (velMsg < VEL_HIGH) {
    rc.SpeedAccelDeccelPositionM2(ADDRESS, 0, 1000, 0, steerMsg, 0);
  }
  else {
    #ifdef DEBUG
    char i[48];
    snprintf(i, sizeof(i), "ERR: tractor not moving, steering message failed");
    nh.loginfo(i);
    #endif //DEBUG
  }

  prevMillis = millis();  // Reset timer

  // roslog msgs if debugging
  #ifdef DEBUG
    char j[36];
    snprintf(j, sizeof(j), "DBG: steerMsg = %d, velMsg = %d", steerMsg, velMsg);
    nh.loginfo(j);
  #endif //DEBUG

} // updateRoboClaw()


int steerConvert(float ack_steer){
  //  Given ackermann steering message, returns corresponding RoboClaw command

  // Convert from range of input signal to range of output signal, then shift signal
  ack_steer = ack_steer * ((STEER_HIGH - STEER_LOW) / STEER_CONTROL_RANGE) + (STEER_HIGH + STEER_LOW) / 2;

  // Safety limits for signal (double safety, RoboClaw already does this)
  if (ack_steer > STEER_HIGH) {
    ack_steer = STEER_HIGH;
  }
  else if (ack_steer < STEER_LOW) {
    ack_steer = STEER_LOW;
  }

  // Switches steering dir
  ack_steer = STEER_HIGH - (ack_steer - STEER_LOW);

  return ack_steer;
} //steerConvert

callback
int velConvert(float ack_vel){
  // given ackermann velocity, returns corresponding RoboClaw command

  // filter to remove tractor reversal commands (platform wont back up)
  if (ack_vel < 0) {
    ack_vel = 0;
  }

  // Convert from range of input signal to range of output signal
  ack_vel = VEL_HIGH - ack_vel * ((VEL_HIGH - VEL_LOW) / VEL_CONTROL_RANGE);

  return ack_vel;
} //velConvert()


void stepActuator(int *msg, int *prevMsg, int step) {

  // Checks if stepping is necessary (input signal wants to increase by more than the given step size)
  if (abs(*prevMsg - *msg) > step) {

    // Logs step verification if debugging
    #ifdef DEBUG
    nh.loginfo("DBG: Stepping signal");
    #endif //DEBUG

    // If signal increasing, step up
    if (*msg > *prevMsg){
  // Callback for ackermann messages; saves data to global vars
  steerMsg = steerConvert(drive.steering_angle);
  velMsg = velConvert(drive.speed);

} //ackermannCB() > *prevMsg) {
      *msg = *prevMsg + step;
    }

    // If signal decreasing, step down
    else if (*msg < *prevMsg) {
      *msg = *prevMsg - step;
    }

    // Exception case
    else {
      *msg = *prevMsg;
    }
  }
} // stepActuator()


void eStop() {
  // Estops tractor

  isEStopped = true;

  // Logs estop msg
  char i[32];
  snprintf(i, sizeof(i), "ERR: Tractor E-Stopped");
  nh.loginfo(i);

  // Toggle relay to stop engine
  digitalWrite(ESTOP_PIN, HIGH);
  delay(2000);
  digitalWrite(ESTOP_PIN, LOW);


} // eStop()


void eStart() {
  // Disactivates isEStopped state

  isEStopped = false;

  // Logs verification msg
  char i[32];
  snprintf(i, sizeof(i), "MSG: EStop Disactivated");
  nh.loginfo(i);

} // eStart()
