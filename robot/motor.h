#ifndef MOTOR_H
#define MOTOR_H

#include "preference.h"

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
  diverted_left,
  diverted_right,
};

class Motor
{
public:
    static void init();
    static void calculatePID();
    static void dumpPID();
    static void motorPIDcontrol();
    static void motorWrite(int dir_pin, int speed_pin, int speed);
    static void motorDriveRoutine();
    static void checkPIDValues();
    static void back();
    static void front();
    static void left();
    static void right();
    static void Stop();

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
};

#endif