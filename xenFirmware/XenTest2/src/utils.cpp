#include <Arduino.h>
#include "utils.h"

void _debugPrint(bool newline, const char* format, va_list args) {
    if (!_debugMode) return;
    char buffer[128]; // Adjust size as needed
    vsnprintf(buffer, sizeof(buffer), format, args);
    if (newline)
        Serial.println(buffer);
    else
        Serial.print(buffer);
}

void _print(const char* format, ...) {
    va_list args;
    va_start(args, format);
    _debugPrint(false, format, args);
    va_end(args);
}

void _print(int d) {
    if (!_debugMode) return;
    Serial.print(d);
}

void _print(float f) {
    if (!_debugMode) return;
    Serial.print(f);
}

void _println(int d) {
    if (!_debugMode) return;
    Serial.println(d);
}

void _println(float f) {
    if (!_debugMode) return;
    Serial.println(f);
}

void _println(const char* format, ...) {
    va_list args;
    va_start(args, format);
    _debugPrint(true, format, args);
    va_end(args);
}
void printBytes(const byte *data, unsigned int size) {
    if (!_debugMode) return;
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