
// Initialize tuning variables

const int PLOW_SERVO_MAX = 155;
const int PLOW_SERVO_MIN = 87;
const int PLOW_SERVO_UP = 130;
const int PLOW_SERVO_DOWN = 87;
const int LED_DELAY = 500;


/* 
 * checkPlow Function
 * DESC: ensures plow servo signal within acceptable range
 * ARGS: plow servo signal
 * RTN: verified signal
 */
int checkPlow(int pos) {

  if (PLOW_SERVO_MIN > pos) {
    pos = PLOW_SERVO_MIN;
  }

  else if (PLOW_SERVO_MAX < pos) {
    pos = PLOW_SERVO_MAX;
  }

  return pos;
}


/* 
 *  convertIR Function
 * DESC: converts sensor reading to distance
 * ARGS: sensor (1 or 2), signal to convert
 * RTN: distance from sensor
 */
int convertIR(byte sensor, int sig) {

  int dist;

  if (sensor == 1) {
    dist = 1; //TODO: Calibrate Sensors
  }

  else if (sensor == 2) {
    dist = 2; //TODO: Calibrate Sensors
  }

  return dist;
}


/* 
 * updateLED Function
 * DESC: checks timer for LED update
 * ARGS: time since last LED update, LED state
 * RTN: new LED state
 */
boolean updateLED(long *timer, boolean state) {
  
  if ((millis() - *timer) > LED_DELAY) {
    state = !state;
    *timer = millis();
  }
  
  return state;
}

/* 
 * updateRoboClaw Function
 * DESC: Sends current velocity and steering vals to RoboClaw; called every ROBOCLAW_UPDATE_RATE ms
 * ARGS: left tread velocity, right tread velocity
 * RTN: none
 * 
*/
/*
void updateRoboClaw(int l_vel, int r_vel) {

  //Calculate step sizes based on fidelity
  int steerStep = (STEER_HIGH - STEER_LOW) / STEER_FIDELITY;
  int velStep = (VEL_HIGH - VEL_LOW) / VEL_FIDELITY;

  stepActuator(&velMsg, &prevVelMsg, velStep);
  stepActuator(&steerMsg, &prevSteerMsg, steerStep);

  prevVelMsg = velMsg;
  prevSteerMsg = steerMsg;
  
  rc.SpeedAccelDeccelPositionM1(address, 100000, 1000, 0, velMsg, 0);
  //rc.SpeedAccelDeccelPositionM2(address, 0, 1000, 0, steerMsg, 0);
  prevMillis = millis();

  #ifdef DEBUG
    char i[32];
    snprintf(i, sizeof(i), "steerMsg = %d, velMsg = %d", steerMsg, velMsg);
    nh.loginfo(i);
  #endif //DEBUG

} //updateRoboClaw()
*/

