#pragma once

#include "Arduino.h"
#include "wheel.h"

// benchmark the RPM of a wheel with an encoder wired to the arduino at the given pins

void wheelBenchmarkSetup(Wheel* benchmarkedWheel, int encoderAPin=13, int encoderBPin=2);
void wheelBenchmarkLoop();