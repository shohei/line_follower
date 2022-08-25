#include <Arduino.h>
#include "led.h"

void Led::init(){
  pinMode(L_LED, OUTPUT);
  pinMode(R_LED, OUTPUT);
}

void Led::L_ON()
{
  analogWrite(L_LED, 100);
  digitalWrite(R_LED, HIGH);
}

void Led::L_ON_BRIGHT()
{
  analogWrite(L_LED, 255);
  digitalWrite(R_LED, 0);
}

void Led::R_ON_BRIGHT()
{
  digitalWrite(L_LED, LOW);
  analogWrite(R_LED, 255);
}

void Led::R_ON()
{
  digitalWrite(L_LED, LOW);
  analogWrite(R_LED, 100);
}

void Led::LR_ON()
{
  analogWrite(L_LED, 100);
  analogWrite(R_LED, 100);
}

void Led::LR_ON_BRIGHT()
{
  digitalWrite(L_LED, HIGH);
  digitalWrite(R_LED, HIGH);
}

void Led::LR_OFF()
{
  digitalWrite(L_LED, LOW);
  digitalWrite(R_LED, LOW);
}

void Led::L_BLINK()
{
  Led::LR_OFF();
  for (int i=0;i<5;i++){
    digitalWrite(L_LED, HIGH);
    delay(20);
    digitalWrite(L_LED, LOW);
    delay(20);
  }
}

void Led::R_BLINK()
{
  Led::LR_OFF();
  for (int i=0;i<5;i++){
    digitalWrite(R_LED, HIGH);
    delay(20);
    digitalWrite(R_LED, LOW);
    delay(20);
  }
}