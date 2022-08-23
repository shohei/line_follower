#include "color_sensor.h"
#include <Arduino.h>
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

static void init_color_sensor()
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
}

static void generate_gamma_table()
{
  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
  for (int i = 0; i < 256; i++)
  {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    Preference *state = Preference::getInstance(); 

    if (commonAnode)
    {
      state->gammatable[i] = 255 - x;
    }
    else
    {
      state->gammatable[i] = x;
    }
    //    Serial.println(gammatable[i]);
  }
}

static void dump_color_values(float red, float green, float blue)
{
  Serial.print(Color::gammatable[(int)red]);
  Serial.print(",");
  Serial.print(Color::gammatable[(int)green]);
  Serial.print(",");
  Serial.print(Color::gammatable[(int)blue]);
  Serial.print(": ");
}

static void read_color_sensor()
a{
  float red, green, blue;
  tcs.setInterrupt(false); // turn on LED
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true); // turn off LED

  Preference *state = Preference::getInstance(); 

  if (state->gammatable[(int)red] < 215 && state->gammatable[(int)blue] > 240 && state->gammatable[(int)green] > 240)
  {
    state->colorStatus = Color::enum_red;
    // Serial.println("red");
  }
  else if (state->gammatable[(int)red] > 240 && state->gammatable[(int)blue] < 220 && state->gammatable[(int)green] > 230)
  {
    state->colorStatus = Color::enum_blue;
    // Serial.println("blue");
  }
  else if (state->gammatable[(int)red] > 240 && state->gammatable[(int)blue] > 230 && state->gammatable[(int)green] < 230)
  {
    state->colorStatus = Color::enum_green;
    // Serial.println("green");
  }
  else
  {
    state->colorStatus = Color::enum_white;
    // Serial.println("white");
  }
}