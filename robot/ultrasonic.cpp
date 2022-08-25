#include "ultrasonic.h"
#include "preference.h"
#include "motor.h"
#include <Arduino.h>

SR04 Ultrasonic::sr04 = SR04(ECHO_PIN, TRIG_PIN);
long distance1;
int myangle;
int pulsewidth;
int val;

void Ultrasonic::init(){
  Ultrasonic::servopulse(SERVO_PIN, 90); // the angle of servo is 90 degree
}

void Ultrasonic::check() {
  distance1 = sr04.Distance();             // obtain the value detected by ultrasonic sensor
  if ((distance1 < 15) && (distance1 > 0)) // if the distance is greater than 0 and less than 10
  {
    Motor::mode = OpMode::avoidance;
  }
}

void Ultrasonic::servopulse(int servopin, int myangle) // the running angle of servo
{
  for (int i = 0; i < 20; i++)
  {
    pulsewidth = (myangle * 11) + 500;
    digitalWrite(servopin, HIGH);
    delayMicroseconds(pulsewidth);
    digitalWrite(servopin, LOW);
    delay(20 - pulsewidth / 1000);
  }
}