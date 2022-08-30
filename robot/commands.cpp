#include "commands.h"
#include "preference.h"
#include "motor.h"
#include <Arduino.h>

void Command::run(const Command commands[], int length)
{
    for (int i = 0; i < length; i++)
    {
        switch (commands[i].name)
        {
        case (CommandName::Forward):
            Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, 100);
            Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, 100);
            delay(commands[i].value);
            break;
        case (CommandName::TurnLeft):
            Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, -200);
            Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, 200);
            delay(commands[i].value);
            break;
        case (CommandName::TurnRight):
            Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, 200);
            Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, -200);
            delay(commands[i].value);
            break;
        }
    }
}

void Command::dump(const Command commands[], int length)
{
    for (int i = 0; i < length; i++)
    {
        Serial.print("Command:");
        Serial.print((int)commands[i].name);
        Serial.print(", Value:");
        Serial.println(commands[i].value);
    }
}