#include <Servo.h>
#include <Wire.h>
#include <MsTimer2.h>

#include "led.h"
#include "color_sensor.h"
#include "motor.h"
#include "line_sensor.h"
#include "preference.h"
#include "ultrasonic.h"

const Motor::Command mycom[] = {
  {Motor::CommandName::Forward, 10},
  {Motor::CommandName::TurnLeft, 20},
  {Motor::CommandName::TurnRight, 30}
};

void setup()
{
  Serial.begin(9600);          // start serial monitor and set baud rate to 9600

  LineSensor::init();
  Motor::init();
  Led::init();
  ColorSensor::init();
  Ultrasonic::init();

  delay(300);

  MsTimer2::set(20, Motor::driveRoutine); // 500ms period
  MsTimer2::start();

  Serial.println("Commands1:");
  Serial.println((int)mycom[0].name);
  Serial.println(mycom[0].value);
  Serial.println("Commands2:");
  Serial.println((int)mycom[1].name);
  Serial.println(mycom[1].value);
  Serial.println("Commands3:");
  Serial.println((int)mycom[2].name);
  Serial.println(mycom[2].value);
}

void loop()
{
  ColorSensor::read();
  delay(60);// This delay is requried for the color sensor

  Motor::checkDeviation();

  //TODO: これ(=pre-programmedな経路の移動)をdriveRoutine()で処理したい.
  //ここのavoidRightPath()の記述を置き換える
  if (Motor::mode == OpMode::avoidance) {
      Motor::avoidRightPath();
  }

  Ultrasonic::check();
}
