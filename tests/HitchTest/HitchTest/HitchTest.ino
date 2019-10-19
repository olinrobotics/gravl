#include <RoboClaw.h>                       // Used for motor controller interface
#include <Arduino.h>                        // Used for Arduino functions

#define RC1_ADDRESS 0x80  // Packet serial mode w/ address 0x80        // In front box, for steering and driving
#define RC2_ADDRESS 0x81  // Packet serial modw w/ address 0x81 //TODO: What is this used for?       // In back of tractor, for hitch
// Timeout below equivalent to 10 ms
#define RC_TIMEOUT 10000

auto rc1_serial = &Serial1;
RoboClaw rc1(rc1_serial, RC_TIMEOUT);
auto rc2_serial = &Serial2;
RoboClaw rc2(rc2_serial, RC_TIMEOUT);

bool valid1,valid2;

// Hitch Actuator Ranges
const int H_ACTUATOR_MAX = 1300; // Retracted Actuator  1300
const int H_ACTUATOR_MIN = 540;  // Extended Actuator   540
const int H_ACTUATOR_CENTER = (H_ACTUATOR_MAX + H_ACTUATOR_MIN) / 2;
const int H_ACTUATOR_RANGE = H_ACTUATOR_MAX-H_ACTUATOR_MIN;
// Encoder
const long ENC_STOP_THRESHOLD = 0.0381; // Threshold of blade accuracy to stop in meters


unsigned int hitchMsg = H_ACTUATOR_CENTER; // Start actuator in center
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  rc1.begin(38400);
  rc2.begin(38400);
  Serial.println("setting speed");
  rc2.SpeedAccelDeccelPositionM2(RC2_ADDRESS, 100000, 1000, 0, 540, 1);
//  rc2.SpeedAccelDeccelPositionM2(RC2_ADDRESS, 0, 300, 0, hitchMsg, 1);
}

void loop() {
//  Serial.println("hello!");
//  // put your main code here, to run repeatedly:
//  rc2.SpeedAccelDeccelPositionM2(RC2_ADDRESS, 100000, 1000, 0, H_ACTUATOR_CENTER, 0);
////  delay(100);
//  rc2.SpeedAccelDeccelPositionM2(RC2_ADDRESS, 100000, 1000, 0, 540, 1);
  Serial.println("here!");
//  long last = millis();
//  while(millis()-last<5000){
//    displayspeed();
//    delay(50);
//  }
//  delay(100);
}




//Display Encoder and Speed for Motor 1
void displayspeed(void)
{
  uint8_t status1,status2;
  bool valid1,valid2;
  int32_t enc1 = rc2.ReadEncM2(RC2_ADDRESS, &status1, &valid1);
  int32_t speed1 = rc2.ReadSpeedM2(RC2_ADDRESS, &status2, &valid2);
  
  if(valid1){
    Serial.print("Encoder1:");
    Serial.print(enc1,DEC);
    Serial.print(" ");
    Serial.print(status1,HEX);
    Serial.print(" ");
  }
  if(valid2){
    Serial.print("Speed1:");
    Serial.print(speed1,DEC);
    Serial.print(" ");
  }
  
  Serial.println();
}
