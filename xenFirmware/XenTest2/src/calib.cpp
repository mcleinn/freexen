#include "xen.h"
#include "calib.h"
#include "midi.h"
#include "utils.h"
#include "mux.h"
#include "adc.h"
#include "dac.h"
#include "sdcard.h"
#include "led.h"

float _threshold_delta = 0.5;
float _zeroVoltage[NUM_KEYS];

int _currentCalibrationKey = 0;
int _calibLoopState = STATE_CALIBRATION_START;
int _gain[NUM_KEYS];
int _offset[NUM_KEYS];
float _maxSwing[NUM_KEYS];
short _polarization[NUM_KEYS];
bool _manualCalibration = false;

int _measureAvgStandard = 32;
int _measureAvgCalibration = 256;

void setupCalibration() {
   if (_debugMode) return;
   bool ok = !_manualCalibration && loadCalibrationCSV();
   _manualCalibration = false;
   
   if (!ok) {    
     _calibLoopState = STATE_CALIBRATION_START;
     _mainLoopState = 2;
   }
}

bool readKeyForCalibration() {
  if (_calibLoopState == STATE_CALIBRATION_START) 
    loopCalibrationStart();
  else if (_calibLoopState == STATE_CALIBRATION_OFF) 
    loopCalibrationOff();
  else if (_calibLoopState == STATE_CALIBRATION_WAIT_ON) 
    loopCalibrationWaitOn();
  else if (_calibLoopState == STATE_CALIBRATION_ON) 
    loopCalibrationOn();
  else if (_calibLoopState == STATE_CALIBRATION_STOP) 
    loopCalibrationStop();
  else if (_calibLoopState == STATE_RUNNING) {
    return true;
  }
  return false;
}

void readKeysNormal() {
    for (int key = _fromKey; key <= _toKey; key++) {
        setKey(key); 
        setDacMain(key, _offset[key]);
        delayMicroseconds(_us_delay_after_dac_set);

        int analogValue = getAdcMain(key);
        float voltage = getAdcVoltage(analogValue); // (analogValue * VREF) / ADC_RESOLUTION; 
        peakDetect(voltage, key);
    }
}

void printCalibration(int key) {
    _println("[%d] Offset=%d (%f V), Gain=%d", 
        key+1, 
        _offset[key], 
        getAdcVoltage(_offset[key]),
        _gain[key] 
        );
    _println("[%d] MaxSwing=%f V, Pol=%d",
        key+1, 
        _maxSwing[key], 
        _polarization[key]);
    _println("[%d] Zero=%f", key+1, _zeroVoltage[key]);
    _println("[%d] Threshold=%f V", key+1, _threshold_delta);
}

void setOffset(int key, int offset) {
    _offset[key] = offset;
}

void setGain(int key, int gain) {
    _gain[key] = gain;
}

void setThreshold(float threshold) {
    _threshold_delta = threshold;
}

void loopCalibrationStart() {
    bool ok = !_manualCalibration && loadCalibrationCSV();
    
    if (ok) { 
      analogReadAveraging(_measureAvgStandard);
      for (int k=0; k<_numberLED; k++)
        LEDStrip->setPixelColor(k, 0, 0, k >= _fromKey && k <= _toKey ? 255 : 0);
  
      _calibLoopState = STATE_RUNNING;
      return;
    }
    _currentCalibrationKey = _fromKey;
    _calibLoopState = STATE_CALIBRATION_OFF;
}
  
void loopCalibrationOff() {
    setKey(_currentCalibrationKey);
    delay(10);
  
    analogReadAveraging(_measureAvgCalibration);
  
    setDacMain(_currentCalibrationKey, 0); // set zeroPoint to center
    delayMicroseconds(_us_delay_after_dac_zero);
  
    int unscaledValue = getAdcUnscaled(_currentCalibrationKey);
    float unscaledVoltage = getAdcVoltage(unscaledValue);
  
    int dacValue = 0;
    float closestError = 0;
    calibrationSweep(dacValue, closestError, _currentCalibrationKey, unscaledVoltage);
    setDacMain(_currentCalibrationKey, dacValue);
  
    //int dacValue = getDacValue(unscaledVoltage);
    //_println("DACValue %d", dacValue);
    //mainDAC(2020, 0); // set zeroPoint to center
    delayMicroseconds(_us_delay_after_dac_set);
  
    int scaledValue = getAdcMain(_currentCalibrationKey);
    float scaledVoltage = getAdcVoltage(scaledValue);
  
    _println("[%d] Unscaled zero: %d %f V, scaled zero: %d %f V", _currentCalibrationKey+1, unscaledValue, unscaledVoltage, scaledValue, scaledVoltage);
  
    _zeroVoltage[_currentCalibrationKey] = scaledVoltage;
    _offset[_currentCalibrationKey] = dacValue;
    //_println("3, calibration Key %d", _currentCalibrationKey);
  
    LEDStrip->setPixelColor(_currentCalibrationKey, 0, 255, 0);
  
    _calibLoopState = STATE_CALIBRATION_WAIT_ON;
  }
  
  void loopCalibrationWaitOn() {
    int currentValue = getAdcMain(_currentCalibrationKey);
    float currentVoltage = getAdcVoltage(currentValue); 
    float swing = fabs(currentVoltage - _zeroVoltage[_currentCalibrationKey]);
  
    if (swing > _threshold_delta) {
        LEDStrip->setPixelColor(_currentCalibrationKey, 255, 255, 255);
        _maxSwing[_currentCalibrationKey] = swing;

        _println("[%d] Over threshold: %f V Swing: %f V", _currentCalibrationKey+1, currentVoltage, swing);
        _calibLoopState = STATE_CALIBRATION_ON;
    }
  }
  
  void loopCalibrationOn() {
    int currentValue = getAdcMain(_currentCalibrationKey);
    float currentVoltage = getAdcVoltage(currentValue); 
    float swing = fabs(currentVoltage - _zeroVoltage[_currentCalibrationKey]);
  
    if (swing > _maxSwing[_currentCalibrationKey])
       _maxSwing[_currentCalibrationKey] = swing;
  
    _polarization[_currentCalibrationKey] = currentVoltage > _zeroVoltage[_currentCalibrationKey] ? 1 : -1;
  
    if (swing < _threshold_delta) {
        float percentageFull = swing / MAX_VOLTAGE * 100;
        _println("[%d] Under threshold: %f V. Max swing: %f V (%f \%) Polarization: %d", 
            _currentCalibrationKey+1, 
            currentVoltage, 
            _maxSwing[_currentCalibrationKey], 
            percentageFull,
            _polarization[_currentCalibrationKey]);

        LEDStrip->setPixelColor(_currentCalibrationKey, 0, 0, 0);
        if (_currentCalibrationKey < _toKey) {
            _currentCalibrationKey++;
            _calibLoopState = STATE_CALIBRATION_OFF;
        } else
            _calibLoopState = STATE_CALIBRATION_STOP;
    }
  }
  
  void loopCalibrationStop() {
    analogReadAveraging(_measureAvgStandard);
    saveCalibrationCSV();
    updateAllLEDs();
  
    _calibLoopState = STATE_RUNNING;
  }
  
  void calibrationSweep(int& bestDacValue, float& closestError, int key, float targetVoltage) {
    int windowSize = 4095; // Initial window size is the full DAC range
    int steps = 16;        // Fixed number of steps in each cycle
    bestDacValue = 0;
  
    do { // Continue until the window is very small
      int startDacValue = max(1000, bestDacValue - windowSize);
      int endDacValue = min(3000, bestDacValue + windowSize);
      closestError = 1e6; // Reset the closest error for this cycle
      
      for (int i = 0; i <= steps; i++) {
        int dacValue = startDacValue + i * (endDacValue - startDacValue) / steps;
        if (dacValue > DAC_RESOLUTION_MAX) break; // Avoid exceeding DAC limits
  
        setDacMain(key, dacValue);
  
        float dacVoltage = getDacVoltage(dacValue);
        int adcValue = getAdcMain(key);
        float adcVoltage = getAdcVoltage(adcValue);
        float error = fabs(adcVoltage - targetVoltage);
  
        _println("DAC Value: %d, DAC Voltage: %.4f, ADC Value: %d, ADC Voltage: %.4f V, Error: %.4f",
             dacValue, dacVoltage, adcValue, adcVoltage, error);
  
        if (error < closestError) {
          closestError = error;
          bestDacValue = dacValue;
        }
      } 
  
      _println("| Window: %d, Best DAC Value: %d, Closest Voltage: %.4f V, Closest Error: %.4f", 
           windowSize, bestDacValue, getDacVoltage(bestDacValue), closestError);
  
      windowSize =  windowSize / 2; // Narrow the window size if results improve
    } while (windowSize >= 1);
  
    Serial.print("X ");
    Serial.println(bestDacValue);
  }

  
void saveCalibrationCSV() {
    char filename[255];
    sprintf(filename, "calib.csv");
    Serial.println(filename);
    SD.remove(filename);
    File configFile = SD.open(filename, FILE_WRITE);
    if (configFile) {
        Serial.println("Saving calibration to CSV...");
        for (int i = 0; i < NUM_KEYS; i++) {
          configFile.print(i);
          configFile.print(",");
          configFile.print(_offset[i]);
          configFile.print(",");
          configFile.print(_gain[i]);
          configFile.print(",");
          configFile.print(_maxSwing[i]);          
          configFile.print(",");
          configFile.println(_polarization[i]);
        }
        configFile.close();
        Serial.println("CSV calibration saved.");
    } else {
        Serial.println("Error opening calibration file for writing.");
    }
}

bool loadCalibrationCSV() {
    // Open "calib.csv" from the SD card
    File configFile = SD.open("calib.csv", FILE_READ);
    if (!configFile) {
        Serial.println("Error opening calibration file for reading.");
        return false;
    }

    Serial.println("Loading calibration from CSV...");

    // Read the file until no more lines
    while (configFile.available()) {
        String line = configFile.readStringUntil('\n');
        line.trim();  // Remove any trailing whitespace or newline
        if (line.length() == 0) {
            continue;  // Skip empty lines
        }

        int lastPos = 0;
        int nextPos;

        // 1) key index
        nextPos = line.indexOf(',', lastPos);
        if (nextPos < 0) {
            continue; // Malformed line
        }
        int key = line.substring(lastPos, nextPos).toInt();
        lastPos = nextPos + 1;

        // Make sure key is valid
        if (key < 0 || key >= NUM_KEYS) {
            continue;
        }

        // 2) _offset[key]
        nextPos = line.indexOf(',', lastPos);
        if (nextPos < 0) {
            continue;
        }
        _offset[key] = line.substring(lastPos, nextPos).toInt();
        lastPos = nextPos + 1;

        // 3) _gain[key]
        nextPos = line.indexOf(',', lastPos);
        if (nextPos < 0) {
            continue;
        }
        _gain[key] = line.substring(lastPos, nextPos).toFloat();
        lastPos = nextPos + 1;

        // 4) _maxSwing[key]
        nextPos = line.indexOf(',', lastPos);
        if (nextPos < 0) {
            continue;
        }
        _maxSwing[key] = line.substring(lastPos, nextPos).toFloat();
        lastPos = nextPos + 1;

        // 5) _polarization[key]
        // This is the last field, so just read the remainder of the line
        _polarization[key] = line.substring(lastPos).toInt();

        // will be determined during bootup, when ADCs are read for the first time
        _zeroVoltage[key] = 0;
    }

    configFile.close();
    Serial.println("CSV calibration loaded.");
    return true;
}


// PEAK DETECT
void peakDetect(float voltage, int key) {
    int board, boardKey;
    // "static" variables keep their numbers between each run of this function
    static int state[NUM_KEYS];  // 0=idle, 1=looking for peak, 2=ignore aftershocks
    static float peak[NUM_KEYS];   // remember the highest reading
    static elapsedMillis msec[NUM_KEYS]; // timer to end states 1 and 2
    static bool playing[NUM_KEYS];
  
    if (_zeroVoltage[key] == 0) {
        _zeroVoltage[key] = voltage;
  
        _println("CAL [%d] Zero=%f V", key+1, _zeroVoltage[key]);
    } 

    float voltageSwing = fabs(_zeroVoltage[key] - voltage); 
    
    switch (state[key]) {
      // IDLE state: wait for any reading is above threshold.  Do not set
      // the threshold too low.  You don't want to be too sensitive to slight
      // vibration.
      case 0:
        if (voltageSwing > _threshold_delta) {
            _print("BEGIN [");
            _print(key);
            _print("] ");
            _print(voltage);
            _print(" OFF: ");
            _println(_zeroVoltage[key]);
  
            peak[key] = voltageSwing;
            msec[key] = 0;
            state[key] = 1;
            playing[key] = false;
        }
        return;
  
      // Peak Tracking state: capture largest reading
      case 1:
        getBoardAndBoardKey(key, board, boardKey);
  
        if (voltageSwing > peak[key]) {
          peak[key] = voltageSwing;  
        }
        
        if (msec[key] >= _peakTrackMillis) {     
          int velocity = map(peak[key], _zeroVoltage[key], _maxSwing[key], 1, 127);
          _println("PEAK [%d] %f %d", key+1, peak[key], velocity);
          if (peak[key] > _maxSwing[key]) {
            _maxSwing[key] = peak[key];
            velocity = map(peak[key], _zeroVoltage[key], _maxSwing[key], 1, 127);
            _println("CAL [%d] Off=%f On=%f", key+1, _zeroVoltage[key], _maxSwing[key]);
          }
          if (!playing[key]) {
            usbMIDI.sendNoteOn(_fields[board][boardKey].Note, velocity, _fields[board][boardKey].Channel);
            playing[key] = true;     
            LEDStrip->setPixelColor(key, 255, 255, 255);
          } else {
            usbMIDI.sendPolyPressure(_fields[board][boardKey].Note, velocity, _fields[board][boardKey].Channel); // Send aftertouch data
          }
          msec[key] = 0;
          state[key] = 2;
        }
        return;
  
      // Ignore Aftershock state: wait for things to be quiet azeroPoint.
      default:
        if (voltageSwing > _threshold_delta) {
            msec[key] = 0; // keep resetting timer if above threshold
            state[key] = 1; 
        } else if (msec[key] > _aftershockMillis) {
            getBoardAndBoardKey(key, board, boardKey);
            usbMIDI.sendNoteOff(_fields[board][boardKey].Note, 0, _fields[board][boardKey].Channel);       
            updateLED(key);
            playing[key] = false;
            state[key] = 0; // go back to idle when
        }
    }
  }