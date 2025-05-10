#ifndef UTILS_H
#define UTILS_H

extern bool _debugMode;

void _print(const char* format, ...);
void _print(int d);
void _print(float f);
void _println(const char* format, ...);
void _println(int d);
void _println(float f);
void _debugPrint(bool newline, const char* format, va_list args);
void printBytes(const byte *data, unsigned int size);
byte mergeBytes(byte low, byte high);

#endif // UTILS_H
