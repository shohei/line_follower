#include "color_sensor.h"
#include <Arduino.h>
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void ColorSensor::init()
{
  if (tcs.begin())
  {
    Serial.println("Found sensor");
  }
  else
  {
    Serial.println("No TCS34725 found ... check your connections");
    while (1)
      ; // halt!
  }
  ColorSensor::generate_gamma_table();
}

void ColorSensor::generate_gamma_table()
{
  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
  for (int i = 0; i < 256; i++)
  {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    // Preference *state = Preference::getInstance(); 

    if (commonAnode)
    {
      ColorSensor::gammatable[i] = 255 - x;
    }
    else
    {
      ColorSensor::gammatable[i] = x;
    }
    //    Serial.println(gammatable[i]);
  }
}

void ColorSensor::dump(float red, float green, float blue)
{
  Serial.print(ColorSensor::gammatable[(int)red]);
  Serial.print(",");
  Serial.print(ColorSensor::gammatable[(int)green]);
  Serial.print(",");
  Serial.print(ColorSensor::gammatable[(int)blue]);
  Serial.print(": ");
}

void ColorSensor::read()
{
  float red, green, blue;
  tcs.setInterrupt(false); // turn on LED
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true); // turn off LED

  if (ColorSensor::gammatable[(int)red] < 215 && ColorSensor::gammatable[(int)blue] > 240 && ColorSensor::gammatable[(int)green] > 240)
  {
    ColorSensor::colorStatus = Color::red;
    // Serial.println("red");
  }
  else if (ColorSensor::gammatable[(int)red] > 240 && ColorSensor::gammatable[(int)blue] < 220 && ColorSensor::gammatable[(int)green] > 230)
  {
    ColorSensor::colorStatus = Color::blue;
    // Serial.println("blue");
  }
  else if (ColorSensor::gammatable[(int)red] > 240 && ColorSensor::gammatable[(int)blue] > 230 && ColorSensor::gammatable[(int)green] < 230)
  {
    ColorSensor::colorStatus = Color::green;
    // Serial.println("green");
  }
  else
  {
    ColorSensor::colorStatus = Color::white;
    // Serial.println("white");
  }
}