
// Initialize tuning variables

int PLOW_SERVO_MAX = 155;
int PLOW_SERVO_MIN = 87;
int PLOW_SERVO_UP = 130;
int PLOW_SERVO_DOWN = 87;
int LED_DELAY = 500;


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

