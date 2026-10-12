#pragma once

class Wheel{
    
private:
    const int motorSpeedPin;
    const int motorDirectionPin;
    const bool flipped; //if front and back direction are flipped

    int speed;

public:

    //CAUTION : when creating a wheel, make sure to bind the encoder pulse (with a global variable)
    //This is kinda ghetto but I couldn't figure out a cleaner way to do it
    Wheel(const int motorSpeedPin, const int motorDirectionPin, const bool flipped);
    
    void SetSpeed(int speed); //speed between -255 and 255, with positive being forwards and negative backwards
    const float GetRPM();
    
    void encoderPulse();

private:
    const void SetDirection(bool forwards);
};