#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

#define INPUT_BUFFER_SIZE 100

// Function prototypes
void parseCommand(char* inputBuffer);

struct Command {
    const char* name;
    void (*handler)(float*, int);  // pointer to function, int array and count
    const char* description;
};

// Forward declarations of handlers
void handleLoop(float* params, int count);
void handleKey(float* params, int count);
void handleChannel(float* params, int count);
void handleCalibrate(float* params, int count);
void handleDAC(float* params, int count);
void handleDPot(float* params, int count);
void handleHelp(float* params, int count);
void handleKeyWithColor(float* params, int count);
void handleSetColorSelected(float* params, int count);
void handleSetColorAll(float* params, int count);
void handleLoadConfig(float* params, int count);
void handleSaveConfig(float* params, int count);
void handleLoadCalib(float* params, int count);
void handleSaveCalib(float* params, int count);
void handleSetupLEDs(float* params, int count);
void handleUpdateLEDs(float* params, int count);
void handleSetupCalib(float* params, int count);
void handleThreshold(float* params, int count);
void handleMux(float* params, int count);
void checkSerial();

extern int _currentCalibrationKey;

#endif // MAIN_H
