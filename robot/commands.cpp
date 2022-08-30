#include "commands.h"
#include "preference.h"
#include "motor.h"
#include "line_sensor.h"
#include "led.h"
#include <Arduino.h>

void Command::run(const Command commands[], int length)
{
    for (int i = 0; i < length; i++)
    {
        switch (commands[i].name)
        {
        case (CommandName::Forward):
            Serial.println("command::forward");
            Led::LR_Blink();
            Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, 100);
            Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, 100);
            if (Command::isLineDetected(commands[i].value, commands[i].isStoppable))
            {
                Serial.println("abort command::forward");
                return;
            }
            break;
        case (CommandName::TurnLeft):
            Serial.println("command::left");
            Led::L_Blink();
            Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, -200);
            Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, 200);
            if (Command::isLineDetected(commands[i].value, commands[i].isStoppable))
            {
                Serial.println("abort command::left");
                return;
            }
            break;
        case (CommandName::TurnRight):
            Serial.println("command::right");
            Led::R_Blink();
            Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, 200);
            Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, -200);
            if (Command::isLineDetected(commands[i].value, commands[i].isStoppable))
            {
                Serial.println("abort command::right");
                return;
            }
            break;
        }
    }
}

bool Command::isLineDetected(int duration_ms, bool isStoppable)
{
    long start = millis();
    while ((millis() - start) < duration_ms)
    {
        LineSensor::updateCurrent();
        LineSensor::updatePrevious();
        if (isStoppable)
        {
            if (LineSensor::cur[0] == 1 ||
                LineSensor::cur[1] == 1 ||
                LineSensor::cur[2] == 1)
            {
                Serial.println("Line detected. Abort avoidance.");
                // turn right and exit avoidance mode
                Motor::write(LEFT_CTRL_PIN, LEFT_PWM_PIN, 200);
                Motor::write(RIGHT_CTRL_PIN, RIGHT_PWM_PIN, -200);
                // Tune here
                delay(200);
                Motor::mode = OpMode::following_line;
                return true;
            }
        }
    }
    Serial.print(duration_ms);
    Serial.println("[ms] delay finished.");
    return false;
}

void Command::dump(const Command commands[], int length)
{
    for (int i = 0; i < length; i++)
    {
        Serial.print("Command:");
        Serial.print((int)commands[i].name);
        Serial.print(", Value:");
        Serial.print(commands[i].value);
        Serial.print(", IsStoppable:");
        Serial.println((int)commands[i].isStoppable);
    }
}