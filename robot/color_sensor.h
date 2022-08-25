#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include "Adafruit_TCS34725.h"
/* Color sensor settings */
#define commonAnode true
/* End of color sensor settings */

enum class Color
{
    red,
    blue,
    green,
    white
};

class ColorSensor
{
public:
    inline static Color colorStatus;
    inline static byte gammatable[256];
    static void init();
    static void generate_gamma_table();
    static void dump(float red, float green, float blue);
    static void read();
};

#endif