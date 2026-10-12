#include "Arduino.h"
#include "wheel.h"
#include "wheelBenchmark.h"

//benchmark the RPM of a wheel with an encoder wired to the arduino at the given pins
//most of this code is taken from the example given by joyIT (the omniwheels manufacturer)


namespace{
    float rpm = 0;
    bool spinningForward = false;
    float time1 = 0;
    float time2 = 0;
    volatile int encoderDownCount = 0;
    int aimedSpeed = 40;
    Wheel *benchmarkedWheel;
    int encoderAPin = -1;
}

void UpdateRPM();
void encoderPulse();

void wheelBenchmarkSetup(Wheel* wheel, int encoderAPin, int encoderBPin){
    benchmarkedWheel = wheel;
    ::encoderAPin = encoderAPin;
    pinMode(encoderAPin, INPUT_PULLUP);
    pinMode(encoderBPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderBPin), encoderPulse, FALLING);
}

void wheelBenchmarkLoop()
{
    if (aimedSpeed >= 255)
    {
        Serial.println("Done");
        aimedSpeed = 0;
        benchmarkedWheel->SetSpeed(0);
    }
    else
        benchmarkedWheel->SetSpeed(aimedSpeed);
    // serialLoop();

    delay(500); // wait for wheel to speed up
    UpdateRPM(); // flush old data
    delay(1000);
    UpdateRPM();
    /*Serial.print(speed);
    Serial.print(',');*/
    Serial.print(rpm);
    Serial.print("\n");

    aimedSpeed += 2;
}

void encoderPulse()
{
    spinningForward = digitalRead(encoderAPin) == LOW;
    encoderDownCount++;
    time2 = millis();
}

void UpdateRPM()
{
    if (time2 > time1)
    {
        // Calculating the rpm of the motor
        rpm = 60000 * encoderDownCount / (time2 - time1);
        rpm = rpm / 234.3; // Gear ratio
        time1 = time2;
        encoderDownCount = 0;
    }
}