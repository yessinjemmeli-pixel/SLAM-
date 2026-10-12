#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <Arduino.h>
#include "orders.h"

// Code using Robust Arduino Serial Protocol: https://github.com/araffin/arduino-robust-serial

void read_signed_bytes(int8_t* buffer, size_t n);
void wait_for_bytes(int num_bytes, unsigned long timeout);

int8_t read_i8();
int16_t read_i16();
void write_i8(int8_t num);
void write_i16(int16_t num);
Order read_order();
void write_order(Order order);