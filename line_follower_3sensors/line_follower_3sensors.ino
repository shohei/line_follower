/*
 keyestudio smart turtle robot
 lesson 10
 Thacking turtle
 http://www.keyestudio.com
*/
#include <Servo.h>

// #define DEBUG
#define L_LED 10
#define R_LED 11
#define PWM_VAL 100

int left_ctrl = 2;       // define direction control pin of A motor
int left_pwm = 9;        // define PWM control pin of A motor
int right_ctrl = 4;      // define direction control pin of B motor
int right_pwm = 5;       // define PWM control pin of B motor
int sensor_l = A4;       // define the pin of left line tracking sensor
int sensor_c = A5;       // define the pin of middle line tracking sensor
int sensor_r = A3;       // define the pin of right line tracking sensor
int l_val, c_val, r_val; // define these variables

Servo leftServo;
Servo rightServo;

const int lineFollowSensor0 = A4;
const int lineFollowSensor1 = A5;
const int lineFollowSensor2 = A3;

int LFSensor[3] = {0, 0, 0};

int mode = 0;

#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2

float Kp = 30;
float Ki = 0.7;
float Kd = 100;

float error = 0, P = 0, I = 0, D = 0, PIDvalue = 0;
float previousError = 0, previousI = 0;

// const int power = 500;
const int iniMotorPower = 100;
const int adj = 1;
// float adjTurn = 8;

int leftMotorSpeed = iniMotorPower;
int rightMotorSpeed = iniMotorPower;

void setup()
{
  Serial.begin(9600);          // start serial monitor and set baud rate to 9600
  pinMode(left_ctrl, OUTPUT);  // set direction control pin of A motor to OUTPUT
  pinMode(left_pwm, OUTPUT);   // set PWM control pin of A motor to OUTPUT
  pinMode(right_ctrl, OUTPUT); // set direction control pin of B motor to OUTPUT
  pinMode(right_pwm, OUTPUT);  // set PWM control pin of B motor to OUTPUT
  pinMode(sensor_l, INPUT);    // set the pins of left line tracking sensor to INPUT
  pinMode(sensor_c, INPUT);    // set the pins of middle line tracking sensor to INPUT
  pinMode(sensor_r, INPUT);    // set the pins of right line tracking sensor to INPUT

  // leftServo.attach(left_pwm);
  // rightServo.attach(right_pwm);

  digitalWrite(left_ctrl, HIGH);
  digitalWrite(right_ctrl, HIGH);

  pinMode(L_LED, OUTPUT);
  pinMode(R_LED, OUTPUT);
}

void loop()
{
  //  tracking(); //run main program
  readLFSsensors();
  // dump();
  if (mode == FOLLOWING_LINE)
  {
    calculatePID();
    motorPIDcontrol();
    // checkPIDvalues();
    dumpPID();
  } else {
    Stop();
  }
}

void checkPIDvalues()
{
  Serial.print("P:");
  Serial.print(Kp*P);
  Serial.print(",I:");
  Serial.print(Ki*I);
  Serial.print(",D:");
  Serial.println(Kd*D);  
}

void dumpPID(){
  Serial.print (PIDvalue);
  Serial.print (" ==> Left, Right:  ");
  Serial.print (leftMotorSpeed);
  Serial.print ("   ");
  Serial.println (rightMotorSpeed);
}

void dump(){
  Serial.print("l_val:");
  Serial.print(LFSensor[0]);
  Serial.print(",c_val:");
  Serial.print(LFSensor[1]);
  Serial.print(",r_val:");
  Serial.println(LFSensor[2]);
}

void readLFSsensors()
{
  LFSensor[0] = digitalRead(lineFollowSensor0);
  LFSensor[1] = digitalRead(lineFollowSensor1);
  LFSensor[2] = digitalRead(lineFollowSensor2);

  if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 1))
  {
    mode = FOLLOWING_LINE;
    error = 2;
    R_ON_BRIGHT();
  }
  else if ((LFSensor[0] == 0) && (LFSensor[1] == 1) && (LFSensor[2] == 1))
  {
    mode = FOLLOWING_LINE;
    error = 1;
    R_ON();
  }
  else if ((LFSensor[0] == 0) && (LFSensor[1] == 1) && (LFSensor[2] == 0))
  {
    mode = FOLLOWING_LINE;
    error = 0;
    LR_ON();
  }
  else if ((LFSensor[0] == 1) && (LFSensor[1] == 1) && (LFSensor[2] == 0))
  {
    mode = FOLLOWING_LINE;
    error = -1;
    L_ON();
  }
  else if ((LFSensor[0] == 1) && (LFSensor[1] == 0) && (LFSensor[2] == 0))
  {
    mode = FOLLOWING_LINE;
    error = -2;
    L_ON_BRIGHT();
  }
  else if ((LFSensor[0] == 1) && (LFSensor[1] == 1) && (LFSensor[2] == 1))
  {
    mode = FOLLOWING_LINE;
    // mode = STOPPED;
    error = 0;
    LR_ON_BRIGHT();
    // LR_OFF();
  }
  else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0))
  {
    mode = NO_LINE;
    error = 0;
    LR_OFF();
  }
}

void L_ON()
{
  analogWrite(L_LED, 100);
  digitalWrite(R_LED, HIGH);
}

void L_ON_BRIGHT()
{
  analogWrite(L_LED, 255);
  digitalWrite(R_LED, 0);
}

void R_ON_BRIGHT()
{
  digitalWrite(L_LED, LOW);
  analogWrite(R_LED, 255);
}

void R_ON()
{
  digitalWrite(L_LED, LOW);
  analogWrite(R_LED, 100);
}

void LR_ON()
{
  analogWrite(L_LED, 100);
  analogWrite(R_LED, 100);
}

void LR_ON_BRIGHT()
{
  digitalWrite(L_LED, HIGH);
  digitalWrite(R_LED, HIGH);
}

void LR_OFF()
{
  digitalWrite(L_LED, LOW);
  digitalWrite(R_LED, LOW);
}

void tracking()
{
  l_val = digitalRead(sensor_l); // read the value of left line tracking sensor
  c_val = digitalRead(sensor_c); // read the value of middle line tracking sensor
  r_val = digitalRead(sensor_r); // read the value of right line tracking sensor
  Serial.print("l_val:");
  Serial.print(l_val);
  Serial.print(",c_val:");
  Serial.print(c_val);
  Serial.print(",r_val:");
  Serial.println(r_val);
  if (c_val == 1) // if the state of middle one is 1, which means detecting black line

  {
    front(); // car goes forward
    LR_ON();
  }
  else
  {
    if ((l_val == 1) && (r_val == 0)) // if only left line tracking sensor detects black trace

    {
      left(); // car turns left
      L_ON();
    }
    else if ((l_val == 0) && (r_val == 1)) /// if only right line tracking sensor detects black trace

    {
      right(); // car turns right
      R_ON();
    }
    else // if none of line tracking sensor detects black line
    {
      Stop(); // car stops
      LR_OFF();
    }
  }
}



void calculatePID()
{
  P = error;
  I = I + error;
  D = error - previousError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;
}


void motorPIDcontrol()
{

  leftMotorSpeed = iniMotorPower + PIDvalue;
  rightMotorSpeed = iniMotorPower * adj - PIDvalue;

  // The motor speed should not exceed the max PWM value
  // constrain(leftMotorSpeed, 1000, 2000);
  // constrain(rightMotorSpeed, 1000, 2000);
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  motorWrite(left_ctrl, left_pwm, leftMotorSpeed);
  motorWrite(right_ctrl, right_pwm, rightMotorSpeed);

  // leftServo.writeMicroseconds(leftMotorSpeed);
  // rightServo.writeMicroseconds(rightMotorSpeed);

}

void motorWrite(int dir_pin, int speed_pin, int speed){
  if (speed > 0){
    digitalWrite(dir_pin, HIGH);
  } else {
    digitalWrite(dir_pin, LOW);
  }
    analogWrite(speed_pin, speed);
}

void back() // define the status of going forward
{
#ifndef DEBUG
  digitalWrite(left_ctrl, LOW);
  analogWrite(left_pwm, PWM_VAL);
  digitalWrite(right_ctrl, LOW);
  analogWrite(right_pwm, PWM_VAL);
#endif
}
void front() // define the state of going back
{
#ifndef DEBUG
  digitalWrite(left_ctrl, HIGH);
  analogWrite(left_pwm, PWM_VAL);
  digitalWrite(right_ctrl, HIGH);
  analogWrite(right_pwm, PWM_VAL);
#endif
}
void left() // define the left-turning state
{
#ifndef DEBUG
  digitalWrite(left_ctrl, LOW);
  analogWrite(left_pwm, PWM_VAL);
  digitalWrite(right_ctrl, HIGH);
  analogWrite(right_pwm, PWM_VAL);
#endif
}
void right() // define the right-turning state
{
#ifndef DEBUG
  digitalWrite(left_ctrl, HIGH);
  analogWrite(left_pwm, PWM_VAL);
  digitalWrite(right_ctrl, LOW);
  analogWrite(right_pwm, PWM_VAL);
#endif
}
void Stop() // define the state of stop
{
  analogWrite(left_pwm, 0);
  analogWrite(right_pwm, 0);
}
//*********************************************************
