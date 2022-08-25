#include "motor.h"
#include "preference.h"
#include <Arduino.h>
#include "line_sensor.h"

int Motor::leftMotorSpeed = INITIAL_MOTOR_POWER;
int Motor::rightMotorSpeed = INITIAL_MOTOR_POWER;

void Motor::init()
{
    pinMode(LEFT_CTRL_PIN, OUTPUT);  // set direction control pin of A motor to OUTPUT
    pinMode(LEFT_PWM_PIN, OUTPUT);   // set PWM control pin of A motor to OUTPUT
    pinMode(RIGHT_CTRL_PIN, OUTPUT); // set direction control pin of B motor to OUTPUT
    pinMode(RIGHT_PWM_PIN, OUTPUT);  // set PWM control pin of B motor to OUTPUT
    digitalWrite(LEFT_CTRL_PIN, HIGH);
    digitalWrite(RIGHT_CTRL_PIN, HIGH);
}

void Motor::calculatePID()
{
  Motor::P = Motor::error;
  Motor::I = Motor::I + Motor::error;
  Motor::D = Motor::error - Motor::previousError;
  Motor::PIDvalue = (Kp * Motor::P) + (Ki * Motor::I) + (Kd * Motor::D);
  Motor::previousError = Motor::error;
}

void Motor::executePIDcontrol()
{
  Motor::leftMotorSpeed = INITIAL_MOTOR_POWER + Motor::PIDvalue;
  Motor::rightMotorSpeed = INITIAL_MOTOR_POWER * ADJ - Motor::PIDvalue;
  // The motor speed should not exceed the max PWM value
  Motor::leftMotorSpeed = constrain(Motor::leftMotorSpeed, -255, 255);
  Motor::Motor::rightMotorSpeed = constrain(Motor::Motor::rightMotorSpeed, -255, 255);

  Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, Motor::leftMotorSpeed);
  Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, Motor::Motor::rightMotorSpeed);
}

void Motor::write(int dir_pin, int speed_pin, int speed)
{
  if (speed > 0)
  {
    digitalWrite(dir_pin, HIGH);
  }
  else
  {
    digitalWrite(dir_pin, LOW);
  }
  analogWrite(speed_pin, speed);
}

char buff[30];
void Motor::driveRoutine()
{
  if (Motor::mode != OpMode::avoidance) {
    LineSensor::read();
  }
  // LineSensor::dump();
  if (Motor::mode == OpMode::following_line)
  {
    Motor::calculatePID();
    Motor::executePIDcontrol();
    // Motor::checkPIDvalues();
    // Motor::dumpPID();
  } else if (Motor::mode == OpMode::recovery) {
   //do nothing 
  } else if (Motor::mode == OpMode::avoidance) {
    //do nothing
  } 
  else if (Motor::mode == OpMode::no_line || Motor::mode == OpMode::stopped)
  {
    if (LineSensor::prev[0]!=LineSensor::cur[0] || 
       LineSensor::prev[1]!=LineSensor::cur[1] || 
      LineSensor::prev[2]!=LineSensor::cur[2]) {
      //The diversion from the line is detected for the first time 
      //Trigger recovery mode 
      if (LineSensor::prev[0]==1 && LineSensor::cur[0] == 0) {
        Motor::failStatus = failMode::diverted_left;
      } else if (LineSensor::prev[2]==1 && LineSensor::cur[2] == 0) {
        Motor::failStatus = failMode::diverted_right;
      }
    }
    Motor::Stop();
    Motor::mode = OpMode::recovery;
  }
  LineSensor::prev[0] = LineSensor::cur[0];
  LineSensor::prev[1] = LineSensor::cur[1];
  LineSensor::prev[2] = LineSensor::cur[2];
}


void checkPIDvalues()
{
  Serial.print("P:");
  Serial.print(Kp * Motor::P);
  Serial.print(",I:");
  Serial.print(Ki * Motor::I);
  Serial.print(",D:");
  Serial.println(Kd * Motor::D);
}

void dumpPID()
{
  Serial.print(Motor::PIDvalue);
  Serial.print(" ==> Left, Right:  ");
  Serial.print(Motor::leftMotorSpeed);
  Serial.print("   ");
  Serial.println(Motor::Motor::rightMotorSpeed);
}

void Motor::back() // define the status of going forward
{
  digitalWrite(LEFT_CTRL_PIN, LOW);
  analogWrite(LEFT_PWM_PIN, PWM_VAL);
  digitalWrite(RIGHT_CTRL_PIN, LOW);
  analogWrite(RIGHT_PWM_PIN, PWM_VAL);
}

void Motor::front() // define the state of going back
{
  digitalWrite(LEFT_CTRL_PIN, HIGH);
  analogWrite(LEFT_PWM_PIN, PWM_VAL);
  digitalWrite(RIGHT_CTRL_PIN, HIGH);
  analogWrite(RIGHT_PWM_PIN, PWM_VAL);
}

void Motor::left() // define the left-turning state
{
  digitalWrite(LEFT_CTRL_PIN, LOW);
  analogWrite(LEFT_PWM_PIN, PWM_VAL);
  digitalWrite(RIGHT_CTRL_PIN, HIGH);
  analogWrite(RIGHT_PWM_PIN, PWM_VAL);
}

void Motor::right() // define the right-turning state
{
  digitalWrite(LEFT_CTRL_PIN, HIGH);
  analogWrite(LEFT_PWM_PIN, PWM_VAL);
  digitalWrite(RIGHT_CTRL_PIN, LOW);
  analogWrite(RIGHT_PWM_PIN, PWM_VAL);
}

void Motor::Stop() // define the state of stop
{
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
}


