#include "wheel.h"
#include <Arduino.h>

Wheel::Wheel(int motorSpeedPin, int motorDirectionPin, bool flipped)
    : motorSpeedPin(motorSpeedPin), motorDirectionPin(motorDirectionPin), flipped(flipped) {
    pinMode(motorSpeedPin, OUTPUT);
    pinMode(motorDirectionPin, OUTPUT);

    SetDirection(true); // set default direction as forwards

    /*pinMode(encoderAPin, INPUT_PULLUP);
    pinMode(encoderBPin, INPUT_PULLUP);*/

    // This is done using a global variable, since you can't attach interrupt with a class method
    // attachInterrupt(digitalPinToInterrupt(encoderBPin), encoderBDownwardsPulse, FALLING);
}

void Wheel::SetSpeed(int speed) {
    if (speed >= 0)
        SetDirection(true);
    else {
        SetDirection(false);
        speed = -speed;
    }

    this->speed = speed;
    analogWrite(motorSpeedPin, speed);
}

const void Wheel::SetDirection(bool forwards) {
    if (flipped)
        forwards = !forwards;
    digitalWrite(motorDirectionPin, forwards ? LOW : HIGH);
}

const float Wheel::GetRPM() {
    // TODO: Implement RPM function
    return 0;
}