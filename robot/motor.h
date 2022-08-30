#ifndef MOTOR_H
#define MOTOR_H

#include "preference.h"
#include "commands.h"

enum class OpMode
{
  stopped,
  following_line,
  no_line,
  recovery,
  avoidance,
};

enum class failMode
{
  no_failure,
  deviated_left,
  deviated_right,
};
class Motor
{
public:
  static void init();
  static void calculatePID();
  static void dumpPID();
  static void executePIDcontrol();
  static void write(int dir_pin, int speed_pin, int speed);
  static void driveRoutine();
  static void checkPIDValues();
  static void Stop();
  static void avoidRightPath();
  static void checkDeviation();

  inline static OpMode mode;
  inline static failMode failStatus;
  inline static float error;
  inline static float P;
  inline static float I;
  inline static float D;
  inline static float PIDvalue;
  inline static float previousError;
  inline static float previousI;
  static int leftMotorSpeed;
  static int rightMotorSpeed;
  static const Command avoidRight[];
  static int avoidRightLength;
};

#endif