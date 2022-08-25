#include <Servo.h>
#include <Wire.h>
#include <MsTimer2.h>

#include "led.h"
#include "color_sensor.h"
#include "motor.h"
#include "line_sensor.h"
#include "preference.h"
#include "ultrasonic.h"

/* Ultrasonic sensor settings */
#include "SR04.h"   //define the library of ultrasonic sensor
#define TRIG_PIN 12 // set the signal input of ultrasonic sensor to D12
#define ECHO_PIN 13 // set the signal output of ultrasonic sensor to D13
SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);
long distance1, distance2, distance3; // define three distance
const int servopin = 3;               // set the pin of servo to D10: OC2B. Pin 11: OC2A
int myangle;
int pulsewidth;
int val;
/* End of ultrasonic sensor settings */

/* Line sensor settings */
int l_val, c_val, r_val; // define these variables


void setup()
{
  Serial.begin(9600);          // start serial monitor and set baud rate to 9600

  LineSensor::init();

  Motor::init();
  Led::init();
  ColorSensor::init();
  ColorSensor::generate_gamma_table();

  servopulse(servopin, 90); // the angle of servo is 90 degree
  delay(300);

  MsTimer2::set(20, Motor::driveRoutine); // 500ms period
  MsTimer2::start();
}

void loop()
{
  ColorSensor::read();
  delay(60);// This delay is requried for the color sensor

  if (Motor::failStatus == failMode::diverted_left) {
      Led::L_BLINK();
      Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, -100);
      Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, 100);
      delay(100);
      Motor::failStatus = failMode::no_failure;
      Motor::mode = OpMode::stopped;
  } else if (Motor::failStatus == failMode::diverted_right) {
      Led::R_BLINK();
      Motor::Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, 100);
      Motor::Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, -100);
      delay(100);
      Motor::failStatus = failMode::no_failure;
      Motor::mode = OpMode::stopped;
  } 

  if (Motor::mode == OpMode::avoidance) {
      avoidRightPath();
  }

  distance1 = sr04.Distance();             // obtain the value detected by ultrasonic sensor
  if ((distance1 < 15) && (distance1 > 0)) // if the distance is greater than 0 and less than 10
  {
    Motor::mode = OpMode::avoidance;
  }
}

void avoidRightPath(){
      //TODO:
      //復帰を検知したら中止してLine following動作をさせたい
      int turn_duration = 520;
      int straight_duration = 2000;
      //turn right
      Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, 200);
      Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, -200);
      delay(turn_duration);
      //straight
      Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, 100);
      Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, 100);
      delay(straight_duration);
      //turn left
      Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, -200);
      Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, 200);
      delay(turn_duration);
      //straight
      Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, 100);
      Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, 100);
      delay(straight_duration);
      //turn left 
      Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, -200);
      Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, 200);
      delay(turn_duration);
      //straight
      Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, 100);
      Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, 100);
      delay(straight_duration);
      //turn right 
      Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, 200);
      Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, -200);
      delay(turn_duration);
}

void servopulse(int servopin, int myangle) // the running angle of servo
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
