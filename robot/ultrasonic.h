#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "SR04.h"   //define the library of ultrasonic sensor

class Ultrasonic
{
public:
    static void init();
    static void servopulse();
    static void servopulse(int servopin, int myangle);
    static void check();
    static SR04 sr04;
};



#endif