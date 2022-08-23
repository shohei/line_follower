#include <Wire.h>
#include "Adafruit_TCS34725.h"
#define redpin 3
#define greenpin 5
#define bluepin 6
#define commonAnode true
byte gammatable[256];
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(9600);
  init_color_sensor();
  init_color_leds();  
  generate_gamma_table();
}

void loop() {
  float red, green, blue;
  tcs.setInterrupt(false);  // turn on LED
  delay(60);  // takes 50ms to read
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);  // turn off LED

  dump_values(red, green, blue);

  emit_leds(red, green, blue);

  if (gammatable[(int)red] < 215 && gammatable[(int)blue] > 240 && gammatable[(int)green] > 240) {
    Serial.println("red");
  } else if (gammatable[(int)red] > 240 && gammatable[(int)blue] < 220 && gammatable[(int)green] > 230) {
    Serial.println("blue");
  } else if (gammatable[(int)red] > 240 && gammatable[(int)blue] > 230 && gammatable[(int)green] < 230) {
    Serial.println("green");
  } else {
    Serial.println("white");
  }
}


void init_color_sensor() {
  if (tcs.begin()) {
    //Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}

void init_color_leds() {
  // use these three pins to drive an LED
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);

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

void emit_leds(float red, float green, float blue) {
  analogWrite(redpin, gammatable[(int)red]);
  analogWrite(greenpin, gammatable[(int)green]);
  analogWrite(bluepin, gammatable[(int)blue]);
}
