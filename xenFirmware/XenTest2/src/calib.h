#include <Adafruit_NeoPixel.h>

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
void readKeysNormal();
void peakDetect(float voltage, int key);
void readKeysNormal();
void loopCalibrationStart();
void loopCalibrationWaitOn();
void loopCalibrationOn();
void loopCalibrationOff();
void loopCalibrationStop();
void calibrationSweep(int& bestDacValue, float& closestError, int key, float targetVoltage);
void saveCalibrationCSV();
bool loadCalibrationCSV();
void peakDetect(float voltage, int key);
void printCalibration(int key);
void setOffset(int key, int offset);
void setGain(int key, int offset);
void setThreshold(float threshold);

extern int _us_delay_after_dac_zero;
extern int _us_delay_after_dac_set;
extern int _fromKey, _toKey, _numberLED;
extern Adafruit_NeoPixel* LEDStrip;

#endif // CALIB_H
