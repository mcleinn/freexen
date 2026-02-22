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
void handleAveraging(float* params, int count);
void handleRun(float* params, int count);
void handlePlot(float* params, int count);
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
void handleMeasureNoise(float* params, int count);
void handleClearCalib(float* params, int count);
void handleFormat(float* params, int count);
void handleMuxDelay(float* params, int count);
void handleReset(float* params, int count);
void handleVersion(float* params, int count);
void handleCalStat(float* params, int count);
void handleReadKey(float* params, int count);
void handleReadStats(float* params, int count);
void handleProblemReport(float* params, int count);
void handleRate(float* params, int count);
void handleSettleTest(float* params, int count);
void handleSettleSweep(float* params, int count);
void handleIdleAudit(float* params, int count);
void handleRuntimePerf(float* params, int count);
void handleTest0(float* params, int count);
void handleTest1(float* params, int count);
void handleTest2(float* params, int count);
void handleTest3(float* params, int count);
void handleTest6(float* params, int count);
void handleMux(float* params, int count);
void checkSerial();
void updateDebugState();
extern int _currentCalibrationKey, _measureAvgStandard;

extern bool _diagActive;

#endif // MAIN_H
