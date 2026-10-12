#include "wheel.h"

// this file isn't actually used by anything, and is just used to test the wheels

void Forwards(Wheel *wheelFrontLeft, Wheel *wheelFrontRight, Wheel *wheelBackLeft, Wheel *wheelBackRight, int speed) {
    wheelFrontLeft->SetSpeed(speed);
    wheelFrontRight->SetSpeed(speed);
    wheelBackRight->SetSpeed(speed);
    wheelBackLeft->SetSpeed(speed);
}

void Backwards(Wheel *wheelFrontLeft, Wheel *wheelFrontRight, Wheel *wheelBackLeft, Wheel *wheelBackRight, int speed) {
    wheelFrontLeft->SetSpeed(-speed);
    wheelFrontRight->SetSpeed(-speed);
    wheelBackRight->SetSpeed(-speed);
    wheelBackLeft->SetSpeed(-speed);
}

void GoLeft(Wheel *wheelFrontLeft, Wheel *wheelFrontRight, Wheel *wheelBackLeft, Wheel *wheelBackRight, int speed) {
    wheelFrontLeft->SetSpeed(-speed);
    wheelFrontRight->SetSpeed(speed);
    wheelBackRight->SetSpeed(-speed);
    wheelBackLeft->SetSpeed(speed);
}

void GoRight(Wheel *wheelFrontLeft, Wheel *wheelFrontRight, Wheel *wheelBackLeft, Wheel *wheelBackRight, int speed) {
    wheelFrontLeft->SetSpeed(speed);
    wheelFrontRight->SetSpeed(-speed);
    wheelBackRight->SetSpeed(speed);
    wheelBackLeft->SetSpeed(-speed);
}

void Stop(Wheel *wheelFrontLeft, Wheel *wheelFrontRight, Wheel *wheelBackLeft, Wheel *wheelBackRight) {
    wheelFrontLeft->SetSpeed(0);
    wheelFrontRight->SetSpeed(0);
    wheelBackRight->SetSpeed(0);
    wheelBackLeft->SetSpeed(0);
}