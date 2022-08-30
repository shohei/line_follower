#include <Servo.h>
#include <Wire.h>
#include <MsTimer2.h>

#include "led.h"
#include "color_sensor.h"
#include "motor.h"
#include "line_sensor.h"
#include "preference.h"
#include "ultrasonic.h"
#include "commands.h"

void setup()
{
  Serial.begin(9600);

  LineSensor::init();
  Motor::init();
  Led::init();
  ColorSensor::init();
  Ultrasonic::init();

  delay(300);

  MsTimer2::set(20, Motor::driveRoutine); // 500ms period
  MsTimer2::start();
  
}

void loop()
{
  ColorSensor::read();
  delay(60);// This delay is requried for the color sensor

  Motor::checkDeviation();

  //TODO: これ(=pre-programmedな経路の移動)をdriveRoutine()で処理したい.
  //ここのavoidRightPath()の記述を置き換える
  if (Motor::mode == OpMode::avoidance) {
      Command::run(Motor::avoidRight, Motor::avoidRightLength);
  }

  Ultrasonic::check();
}
