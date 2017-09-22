#include <Servo.h>
#define led1 2
#define servo_num1 9
Servo servo_steer;
unsigned long time = 0, interval = 200;
byte led_state = LOW; 
int in_angle, out_angle;

void setup() {
// put your setup code here, to run once:
servo_steer.attach(servo_num1);
pinMode(led1, OUTPUT);
time = millis();
}

void loop() {
// put your main code here, to run repeatedly:
if(millis() - time > interval){
if(!led_state){
led_state = HIGH;
}else if(led_state){
led_state = LOW;
}
digitalWrite(led1, led_state);
time = millis();
}
/*in_angle = analogRead(A0);
out_angle = map(in_angle, 0, 255, 1, 178);*/
//the rightmost angle from servo is 0
//the left most angle for servo is ~160
servo_steer.write(0);
}​
