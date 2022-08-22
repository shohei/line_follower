/*
 keyestudio smart turtle robot
 lesson 10
 Thacking turtle
 http://www.keyestudio.com
*/
#include <Servo.h>
#include <Wire.h>
#include <MsTimer2.h>
#include "Adafruit_TCS34725.h"

/* Color sensor settings */
#define commonAnode true
byte gammatable[256];
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
/* End of color sensor settings */

// #define DEBUG
#define L_LED 6 //Timer 0A
#define R_LED 10 //Timer 1B
#define PWM_VAL 100

int left_ctrl = 2;       // define direction control pin of A motor
int left_pwm = 9;        // define PWM control pin of A motor: Timer 1A
int right_ctrl = 4;      // define direction control pin of B motor
int right_pwm = 5;       // define PWM control pin of B motor: Timer 0B
int sensor_l = A1;       // define the pin of left line tracking sensor
int sensor_c = A2;       // define the pin of middle line tracking sensor
int sensor_r = A3;       // define the pin of right line tracking sensor

/* Ultrasonic sensor settings */
#include "SR04.h"         //define the library of ultrasonic sensor
#define TRIG_PIN 12       // set the signal input of ultrasonic sensor to D12
#define ECHO_PIN 13       // set the signal output of ultrasonic sensor to D13
SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);
long distance1, distance2, distance3; // define three distance
const int servopin = 3;               // set the pin of servo to D10: OC2B. Pin 11: OC2A
int myangle;
int pulsewidth;
int val;
/* End of ultrasonic sensor settings */

/* Line sensor settings */
int l_val, c_val, r_val; // define these variables
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

enum color_enum {
  enum_red,
  enum_blue,
  enum_green,
  enum_white 
};
color_enum colorStatus = enum_white;

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

  digitalWrite(left_ctrl, HIGH);
  digitalWrite(right_ctrl, HIGH);

  pinMode(L_LED, OUTPUT);
  pinMode(R_LED, OUTPUT);

  init_color_sensor();
  generate_gamma_table();

  MsTimer2::set(20, motorDriveRoutine); // 500ms period
  MsTimer2::start();
}

void loop()
{
  read_color_sensor();
  delay(60);
}

void motorDriveRoutine(){
  readLFSsensors();
  dump();
  if (mode == FOLLOWING_LINE)
  {
    calculatePID();
    motorPIDcontrol();
    // checkPIDvalues();
    // dumpPID();
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
  Serial.print(LFSensor[2]);
  Serial.print(",color:");
  Serial.println(colorStatus);
}

void readLFSsensors()
{
  LFSensor[0] = digitalRead(sensor_l);
  LFSensor[1] = digitalRead(sensor_c);
  LFSensor[2] = digitalRead(sensor_r);

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
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  motorWrite(left_ctrl, left_pwm, leftMotorSpeed);
  motorWrite(right_ctrl, right_pwm, rightMotorSpeed);

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

void init_color_sensor() {
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}

void generate_gamma_table() {
  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
  for (int i = 0; i < 256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
    //    Serial.println(gammatable[i]);
  }
}

void dump_values(float red, float green, float blue) {
  Serial.print(gammatable[(int)red]);
  Serial.print(",");
  Serial.print(gammatable[(int)green]);
  Serial.print(",");
  Serial.print(gammatable[(int)blue]);
  Serial.print(": ");
}

void read_color_sensor(){
  float red, green, blue;
  tcs.setInterrupt(false);  // turn on LED
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);  // turn off LED

  if (gammatable[(int)red] < 215 && gammatable[(int)blue] > 240 && gammatable[(int)green] > 240) {
    colorStatus = enum_red;
    // Serial.println("red");
  } else if (gammatable[(int)red] > 240 && gammatable[(int)blue] < 220 && gammatable[(int)green] > 230) {
    colorStatus = enum_blue;
    // Serial.println("blue");
  } else if (gammatable[(int)red] > 240 && gammatable[(int)blue] > 230 && gammatable[(int)green] < 230) {
    colorStatus = enum_green;
    // Serial.println("green");
  } else {
    colorStatus = enum_white;
    // Serial.println("white");
  }
}
