#ifndef LED_H
#define LED_H

#define L_LED 6  // Timer 0A
#define R_LED 10 // Timer 1B

class Led {
    public:
    static void init();
    static void L_ON();
    static void L_ON_BRIGHT();
    static void R_ON();
    static void R_ON_BRIGHT();
    static void LR_ON();
    static void LR_ON_BRIGHT();
    static void LR_OFF();
    static void L_Blink();
    static void R_Blink();
    static void LR_Blink();
};

#endif
