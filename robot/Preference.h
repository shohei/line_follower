#ifndef PREFERENCE_H
#define PREFERENCE_H

#include "color_sensor.h"

class Preference 
{
  private:
    Preference(){};
    virtual ~Preference(){};	
    static Preference* state;
  public:	
    static Preference* getInstance(void){
      return state;
    };
    color_enum colorStatus;
    byte gammatable[256];
};

#endif