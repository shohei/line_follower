#ifndef LED_H
#define LED_H

#define L_LED 6  // Timer 0A
#define R_LED 10 // Timer 1B
#define PWM_VAL 100

void L_ON();
void L_ON_BRIGHT();
void R_ON();
void R_ON_BRIGHT();
void LR_ON();
void LR_ON_BRIGHT();
void LR_OFF();
void L_BLINK();
void R_BLINK();

#endif