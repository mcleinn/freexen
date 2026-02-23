#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "xen.h"

#ifndef CALIB_H
#define CALIB_H

// CALIB
#define MAX_VOLTAGE          3.3

const int _peakTrackMillis = 12;
const int _aftershockMillis = 25; // aftershocks & vibration reject

#define STATE_RUNNING 0
#define STATE_CALIBRATION_START 1
#define STATE_CALIBRATION_OFF 2
#define STATE_CALIBRATION_WAIT_ON 3
#define STATE_CALIBRATION_ON 4
#define STATE_CALIBRATION_STOP 5

void setupCalibration();
bool readKeyForCalibration();
void scanKeysNormal();
void peakDetect(float voltage, int key);
void scanKeysNormal();
void loopCalibrationStart();
void loopCalibrationWaitOn();
void loopCalibrationOn();
void loopCalibrationOff();
void loopCalibrationStop();
void saveCalibrationCSV();
bool loadCalibrationCSV();
void peakDetect(float voltage, int key);
void printCalibration(int key);
void setThreshold(int key, float threshold);
void scanNoise(int ms);
void hsvToRgb(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b);
void showNoise(int adc);
void printNoiseLevels();
void clearCalibration(int fromKey, int toKey);

// Dumps the last calibration run issues (RAM state)
void printLastCalibrationIssues();

// Output format set by main.
extern int _outputFormat;

// Boot/debug messaging flags (set in setupCalibration / main)
extern bool _calibAutoLoadOk;
extern bool _bootedInDebugMode;

// True if calibration arrays were modified since last load/save.
extern bool _calibDirty;

extern int _us_delay_after_dac_zero;
extern int _us_delay_after_dac_set;
extern int _us_delay_after_mux;
extern int _fromKey, _toKey, _numberLED;
extern Adafruit_NeoPixel* LEDStrip;

extern float _threshold_delta[NUM_KEYS];
// Autotune threshold overlay (RAM): >0 overrides calibration threshold.
extern float _autotune_threshold_delta[NUM_KEYS];
float getEffectiveThresholdDelta(int key);
extern float _zeroVoltage[NUM_KEYS];
extern bool _hasZero[NUM_KEYS];
extern float _maxSwing[NUM_KEYS];
extern short _polarization[NUM_KEYS];
extern float _noiseUnscaled[NUM_KEYS];

// Runtime instrumentation (optional)
extern volatile uint32_t _scanPasses;
extern volatile uint32_t _scanKeyReads;

#endif // CALIB_H
