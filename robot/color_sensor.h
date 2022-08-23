#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include "Adafruit_TCS34725.h"
/* Color sensor settings */
#define commonAnode true
/* End of color sensor settings */
class Color
{
public:
    enum color_enum
    {
        enum_red,
        enum_blue,
        enum_green,
        enum_white
    };
    static color_enum colorStatus;
    static byte gammatable[256];
    static void init_color_sensor();
    static void generate_gamma_table();
    static void dump_color_values(float red, float green, float blue);
    static void read_color_sensor();
};

#endif