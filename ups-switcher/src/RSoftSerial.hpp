#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>
class RSoftSerial : public SoftwareSerial {
public:
    RSoftSerial(uint8_t rxPin) : SoftwareSerial(rxPin, -1) {}

    // Override the write method to prevent sending data
    size_t write(uint8_t) override {
        return 0; // Do nothing and return 0
    }

    // Override the write method for buffer input
    size_t write(const uint8_t* buffer, size_t size) override {
        return 0; // Do nothing and return 0
    }
};
