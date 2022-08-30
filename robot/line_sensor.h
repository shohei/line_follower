#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

class LineSensor
{
public:
    static void init();
    static void read();
    static void dump();
    static void updateCurrent();
    static void updatePrevious();
    inline static int cur[3];
    inline static int prev[3];
};

#endif