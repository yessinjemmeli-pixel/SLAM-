#pragma once

/*-----------------------------------------------*/
// Pin assignment (Motors)
/*-----------------------------------------------*/

//For direction pins: they are set with digitalWrite (LOW for forwards, HIGH for backwards)
//For speed pins: they are set using analogWrite on PMW pins. Speed is set from 0 to 255,
//though the wheels won't spin before a speed of about 50 since the motor doesn't give enough power (i think)
const int backLeftMotorDirectionPin = 12;
const int backLeftMotorSpeedPin = 11;
const int frontRightMotorDirectionPin = 7;
const int frontRightMotorSpeedPin = 6;
const int frontLeftMotorDirectionPin = 8;
const int frontLeftMotorSpeedPin = 9;
const int backRightMotorDirectionPin = 4;
const int backRightMotorSpeedPin = 5;