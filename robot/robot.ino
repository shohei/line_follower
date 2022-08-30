#include <Servo.h>
#include <Wire.h>
#include <MsTimer2.h>

#include "led.h"
#include "color_sensor.h"
#include "motor.h"
#include "line_sensor.h"
#include "preference.h"
#include "ultrasonic.h"

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
}

void loop()
{
  ColorSensor::read();
  delay(60);// This delay is requried for the color sensor

  Motor::checkDeviation();

  if (Motor::mode == OpMode::avoidance) {
      Motor::avoidRightPath();
  }

  Ultrasonic::check();
}
