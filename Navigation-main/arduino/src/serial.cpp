#include <Arduino.h>
#include <stdint.h>
#include "orders.h"
// Code using Robust Arduino Serial Protocol: https://github.com/araffin/arduino-robust-serial

// NOTE : Serial.readBytes is SLOW
// this one is much faster, but has no timeout
void read_signed_bytes(int8_t *buffer, uint32_t n) {
    uint32_t i = 0;
    int c;
    while (i < n) {
        c = Serial.read();
        if (c < 0)
            break;
        *buffer++ = (int8_t)c; // buffer[i] = (int8_t)c;
        i++;
    }
}

void wait_for_bytes(int num_bytes, unsigned long timeout) {
    unsigned long startTime = millis();
    // Wait for incoming bytes or exit if timeout
    while ((Serial.available() < num_bytes) && (millis() - startTime < timeout)) {
    }
}

int8_t read_i8() {
    wait_for_bytes(1, 100); // Wait for 1 byte with a timeout of 100 ms
    return (int8_t)Serial.read();
}

int16_t read_i16() {
    int8_t buffer[2] = {0, 0};
    wait_for_bytes(2, 100); // Wait for 2 bytes with a timeout of 100 ms
    read_signed_bytes(buffer, 2);
    return (((int16_t)buffer[0]) & 0x00ff) | (((int16_t)buffer[1]) << 8 & 0xff00);
}

void write_i8(int8_t num) {
    Serial.write(num);
}

void write_i16(int16_t num) {
    int8_t buffer[2] = {(int8_t)(num & 0xff), (int8_t)(num >> 8)};
    Serial.write((uint8_t *)&buffer, 2 * sizeof(int8_t));
}

Order read_order()
{
    return (Order)read_i8();
}

void write_order(Order order)
{
    write_i8((int8_t)order);
}