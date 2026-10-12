#include "constants.h"
#include "orders.h"
#include "serial.h"
#include "wheel.h"
#include <Arduino.h>

#define BAUDRATE 115200

Wheel *wheelFrontLeft;
Wheel *wheelFrontRight;
Wheel *wheelBackRight;
Wheel *wheelBackLeft;
bool connectedToRaspi = false;

unsigned long lostConnectionTime = 0;
const float timeout = 2.0;

void serialLoop();

void setup() {
    Serial.begin(BAUDRATE);

    wheelFrontLeft = new Wheel(frontLeftMotorSpeedPin, frontLeftMotorDirectionPin, false);
    wheelFrontRight = new Wheel(frontRightMotorSpeedPin, frontRightMotorDirectionPin,true);
    wheelBackRight = new Wheel(backRightMotorSpeedPin, backRightMotorDirectionPin, false);
    wheelBackLeft = new Wheel(backLeftMotorSpeedPin, backLeftMotorDirectionPin, false);
}

void loop() {
    serialLoop();
}

void serialLoop(){
    if (Serial.available() <= 0){

        if(lostConnectionTime == 0)
            lostConnectionTime = millis();

        if (millis() - lostConnectionTime > timeout * 1000){
            wheelFrontLeft->SetSpeed(0);
            wheelFrontRight->SetSpeed(0);
            wheelBackRight->SetSpeed(0);
            wheelBackLeft->SetSpeed(0);
        }
        
        return;
    }

    lostConnectionTime = 0;

    // The first byte received is the instruction
    Order order_received = read_order();

    switch (order_received)
    {
        case Order::Hello:
        {
            if (!connectedToRaspi)
            {
                connectedToRaspi = true;
                write_order(Order::Hello);
            }
            else
                write_order(Order::AlreadyConnected);

            break;
        }
        case Order::AlreadyConnected:
        {
            connectedToRaspi = true;
            break;
        }
        case Order::WheelSpeeds:
        {
            int frontLeftWheelSpeed = (int)read_i16();
            int frontRightWheelSpeed = (int)read_i16();
            int backRightWheelSpeed = (int)read_i16();
            int backLeftWheelSpeed = (int)read_i16();

            wheelFrontLeft->SetSpeed(frontLeftWheelSpeed);
            wheelFrontRight->SetSpeed(frontRightWheelSpeed);
            wheelBackLeft->SetSpeed(backLeftWheelSpeed);
            wheelBackRight->SetSpeed(backRightWheelSpeed);
            break;
        }
    }
}

