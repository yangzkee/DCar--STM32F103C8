#ifndef TEST_SOFTWARE_SERIAL_H
#define TEST_SOFTWARE_SERIAL_H

#include "Arduino.h"

class SoftwareSerial : public Stream {
public:
    SoftwareSerial(uint8_t, uint8_t) {}
    void begin(long) {}
};

#endif
