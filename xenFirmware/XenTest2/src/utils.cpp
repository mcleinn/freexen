#include <Arduino.h>
#include "utils.h"

void _println(const char* format, ...) {
    char buffer[128]; // Adjust buffer size as needed
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial.println(buffer);
}

void printBytes(const byte *data, unsigned int size) {
    while (size > 0) {
        byte b = *data++;
        if (b < 16) Serial.print('0');
        Serial.print(b, HEX);
        if (size > 1) Serial.print(' ');
        size = size - 1;
    }
}

byte mergeBytes(byte low, byte high) {
    return (low & 0x7F) | ((high & 0x7F) << 7);
}