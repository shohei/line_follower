#include "line_sensor.h"
#include "preference.h"
#include "motor.h"
#include <Arduino.h>
#include "led.h"
#include "color_sensor.h"

void LineSensor::init()
{
    pinMode(LEFT_CTRL_PIN, OUTPUT);  // set direction control pin of A motor to OUTPUT
    pinMode(LEFT_PWM_PIN, OUTPUT);   // set PWM control pin of A motor to OUTPUT
    pinMode(RIGHT_CTRL_PIN, OUTPUT); // set direction control pin of B motor to OUTPUT
    pinMode(RIGHT_PWM_PIN, OUTPUT);  // set PWM control pin of B motor to OUTPUT
}

void LineSensor::read()
{
  LineSensor::updateCurrent();

  if ((LineSensor::cur[0] == 0) && (LineSensor::cur[1] == 0) && (LineSensor::cur[2] == 1))
  {
    Motor::mode = OpMode::following_line;
    Motor::error = 2;
    Led::R_ON_BRIGHT();
  }
  else if ((LineSensor::cur[0] == 0) && (LineSensor::cur[1] == 1) && (LineSensor::cur[2] == 1))
  {
    Motor::mode = OpMode::following_line;
    Motor::error = 1;
    Led::R_ON();
  }
  else if ((LineSensor::cur[0] == 0) && (LineSensor::cur[1] == 1) && (LineSensor::cur[2] == 0))
  {
    Motor::mode = OpMode::following_line;
    Motor::error = 0;
    Led::LR_ON();
  }
  else if ((LineSensor::cur[0] == 1) && (LineSensor::cur[1] == 1) && (LineSensor::cur[2] == 0))
  {
    Motor::mode = OpMode::following_line;
    Motor::error = -1;
    Led::L_ON();
  }
  else if ((LineSensor::cur[0] == 1) && (LineSensor::cur[1] == 0) && (LineSensor::cur[2] == 0))
  {
    Motor::mode = OpMode::following_line;
    Motor::error = -2;
    Led::L_ON_BRIGHT();
  }
  else if ((LineSensor::cur[0] == 1) && (LineSensor::cur[1] == 1) && (LineSensor::cur[2] == 1))
  {
    Motor::mode = OpMode::following_line;
    Motor::error = 0;
    Led::LR_ON_BRIGHT();
  }
  else if ((LineSensor::cur[0] == 0) && (LineSensor::cur[1] == 0) && (LineSensor::cur[2] == 0))
  {
    if (Motor::mode != OpMode::recovery) {
      Motor::mode = OpMode::no_line;
    }
    Motor::error = 0;
    Led::LR_OFF();
  }
}

void LineSensor::updateCurrent(){
  LineSensor::cur[0] = digitalRead(SENSOR_LEFT);
  LineSensor::cur[1] = digitalRead(SENSOR_CENTER);
  LineSensor::cur[2] = digitalRead(SENSOR_RIGHT);
}

void LineSensor::updatePrevious(){
  LineSensor::prev[0] = LineSensor::cur[0];
  LineSensor::prev[1] = LineSensor::cur[1];
  LineSensor::prev[2] = LineSensor::cur[2];
}

void LineSensor::dump()
{
  Serial.print("l_val:");
  Serial.print(LineSensor::cur[0]);
  Serial.print(",c_val:");
  Serial.print(LineSensor::cur[1]);
  Serial.print(",r_val:");
  Serial.print(LineSensor::cur[2]);
  Serial.print(",color:");
  Serial.println((int)ColorSensor::colorStatus);
}
