#ifndef PREFERENCE_H
#define PREFERENCE_H


#define LEFT_CTRL_PIN 2  // define direction control pin of A motor
#define LEFT_PWM_PIN 9   // define PWM control pin of A motor: Timer 1A
#define RIGHT_CTRL_PIN 4 // define direction control pin of B motor
#define RIGHT_PWM_PIN 5  // define PWM control pin of B motor: Timer 0B
#define SENSOR_LEFT A1  // define the pin of left line tracking sensor
#define SENSOR_CENTER A2  // define the pin of middle line tracking sensor
#define SENSOR_RIGHT A3  // define the pin of right line tracking sensor

#define Kp 30.0
#define Ki 0.7
#define Kd 100.0
#define INITIAL_MOTOR_POWER 100
#define ADJ 1

#define PWM_VAL 100

class Preference
{
  private:
    Preference(){};
    virtual ~Preference();
    static Preference *state;
  public:
    static Preference* getInstance(void){
      return state;
    };
};

#endif

