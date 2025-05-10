#include <Arduino.h>
#include <SPI.h>
#include <Math.h>
#include "MCP_DAC.h"

#include "main.h"
#include "sdcard.h"
#include "adc.h"
#include "dpot.h"
#include "dac.h"
#include "mux.h"
#include "xen.h"
#include "led.h"
#include "utils.h"
#include "midi.h"
#include "calib.h"

bool _debugMode = false;
int _mainLoopState = 0;
int _currentKey = 0;
int _ms_plot_mode_delay = 100;

extern int _manualCalibration;

// Command alias table
Command commandTable[] = {
  {"l", handleLoop, "Set loop mode (0 = pause, 1 = plot, 2 = calib, 3 = normal)"},
  {"k", handleKey, "Set key index (no arg for all, otherwise key or from,to)"},
  {"m", handleMux, "Set mux"},
  {"kc", handleKeyWithColor, "Set key index (no arg for all, otherwise key or from,to)"},
  {"o", handleDAC, "Set DAC value (0 to 4095), optional 2nd param = channel"},
  {"g", handleDPot, "Set digital potentiometer value (0 to 255), optional 2nd param = channel"},
  {"t", handleThreshold, "Set threshold"},
  {"c", handleSetColorSelected, "Set color for selected keys (three parameters 0-255)"},
  {"ca", handleSetColorAll, "Set color for all keys (three parameters 0-255)"},
  {"x", handleCalibrate, "Enter calibration mode"},
  {"lconf", handleLoadConfig, "Load configuration (optional: program ID)"},
  {"sconf", handleSaveConfig, "Save configuration (optional: program ID)"},
  {"lcalib", handleLoadCalib, "Load calibration"},
  {"scalib", handleSaveCalib, "Save calibration"},
  {"sled", handleSetupLEDs, "Setup LEDs, optional parameter: number of LEDs (0 to max Key)"},
  {"uled", handleUpdateLEDs, "Update all LEDs with the current configuration"},
  {"h", handleHelp, "Show this help message"},
};

const int commandCount = sizeof(commandTable) / sizeof(Command);

void setup() {
  Serial.begin(115200);

  unsigned long timeout = millis();

  // Wait a short time for serial monitor to connect (or timeout)
  while (!Serial && millis() - timeout < 2000);

  if (Serial) {
    _debugMode = true;
    Serial.println("Debug mode enabled.");
    _mainLoopState = 0;
    _program = 9;
  } else {
    _mainLoopState = 3;
    _program = 5;
  }

  handleHelp(nullptr, 0); 
   
  setupMux();
  setupADC();
  setupDAC();
  setupDPot();
  setupCard();
  setupMidi();
  setupCalibration();

  setupLEDs(280);
  loadConfigurationCSV();
  updateAllLEDs();

  loadCalibrationCSV();

  Serial.println("Ready to receive commands (e.g. a123)...");
}

void loop() {
  if (_debugMode)
    checkSerial();

  if (_mainLoopState == 1) {
    // Plotter mode
    _print("Min:0.0,");
    for (int k=_fromKey; k<= _toKey; k++) {
      setKey(k);
      int adcMain1 = getAdcMain(k);
      int adcUnscaled1 = getAdcUnscaled(k);

      float scaledVoltage1 = getAdcVoltage(adcMain1);
      float unscaledVoltage1 = getAdcVoltage(adcUnscaled1);

      //_print("S%d:%f,U%d:%f,", k, scaledVoltage1, k, unscaledVoltage1);
      _print("S%d:%f,", k+1, scaledVoltage1);
    }
    _println("Max:3.3");
    delay(_ms_plot_mode_delay);
  } else if (_mainLoopState == 2) {
    // Calibration mode
    bool finished = readKeyForCalibration();
    if (finished) {
      if (!_debugMode) 
        _mainLoopState = 3; // Returning to normal loop mode
      else { 
        _mainLoopState = 0;
        Serial.println("Calibration finished. Returning to loop state 0.");
      }
    }
  } else if (_mainLoopState == 3) {
    // Normal operation mode
    readKeysNormal();
  }

  LEDStrip->show();
}

void checkSerial() {
  static char inputBuffer[INPUT_BUFFER_SIZE];
  static int inputIndex = 0;
  
  while (Serial.available() > 0) {
    char incomingByte = (char)Serial.read();
    
    if (incomingByte == '\n') {
      // Terminate the string
      inputBuffer[inputIndex] = '\0';
      // Process the command
      parseCommand(inputBuffer);
      // Reset the buffer index for the next command
      inputIndex = 0;
    } else {
      // Only add character if there is room in the buffer
      if (inputIndex < INPUT_BUFFER_SIZE - 1) {
        inputBuffer[inputIndex++] = incomingByte;
      }
      // Optionally, handle buffer overflow if needed
    }
  }
}

void parseCommand(char* inputBuffer) {
  int len = strlen(inputBuffer);
  if (len < 1) return;

  // Extract command name: all leading letters
  int i = 0;
  while (i < len && isAlpha(inputBuffer[i])) i++;

  if (i == 0) {
    Serial.println("Invalid command: no letters found.");
    return;
  }

  char cmd[INPUT_BUFFER_SIZE] = {0};
  strncpy(cmd, inputBuffer, i);
  cmd[i] = '\0';

  // Extract comma-separated float parameters
  float params[10]; // Max 10 parameters
  int paramCount = 0;

  char* paramStr = inputBuffer + i;
  char* token = strtok(paramStr, ",");

  while (token != NULL && paramCount < 10) {
    params[paramCount++] = atof(token);  // Use atof instead of atoi
    token = strtok(NULL, ",");
  }

  // Find matching command
  for (int j = 0; j < commandCount; j++) {
    if (strcmp(cmd, commandTable[j].name) == 0) {
      commandTable[j].handler(params, paramCount);
      return;
    }
  }

  Serial.print("Unknown command: ");
  Serial.println(cmd);
}


void handleLoop(float* params, int count) {
  if (count >= 1) {
    if (_mainLoopState == 2) {
      _calibLoopState = STATE_CALIBRATION_STOP;
      Serial.print("Cancelling calibration.");
      return;
    }
    _mainLoopState = (int)params[0];
    if (_mainLoopState == 2) { 
      _manualCalibration = true;
      _calibLoopState = STATE_CALIBRATION_START;
    }
    Serial.print("Main loop set to ");
    Serial.println(_mainLoopState);
  }
}

void handleCalibrate(float* params, int count) {
  if (_mainLoopState == 2) {
    _calibLoopState = STATE_CALIBRATION_STOP;
    Serial.print("Cancelling calibration.");
    return;
  }
  _mainLoopState = 2;
  _manualCalibration = true;
  _calibLoopState = STATE_CALIBRATION_START;
  Serial.print("Entered calibration loop.");
}

void handleKey(float* params, int count) {
  if (count == 0) {
    setKeyInterval(0, NUM_KEYS-1);
    _println("Selected keys from %d to %d", _fromKey+1, _toKey+1);
  }
  if (count == 1) {
    setKeyInterval((int)params[0]-1, (int)params[0]-1);
    _println("Set key to %d", _fromKey+1);
    printCalibration(_fromKey);
  }
  if (count == 2) {
    setKeyInterval((int)params[0]-1, (int)params[1]-1);
    _println("Selected keys from %d to %d", _fromKey+1, _toKey+1);
  }
  _currentCalibrationKey = _fromKey;
  _calibLoopState = STATE_CALIBRATION_START;
}

void handleKeyWithColor(float* params, int count) {
  handleKey(params, count);
  
  setColorAll(0, 0, 0);
  for (int k=_fromKey; k <= _toKey; k++) 
    setColor(k, 0, 0, 255);

  updateAllLEDs();
}

void handleDAC(float* params, int count) {
  if (count >= 1) {
    int value = (int)params[0];
    setOffset(_currentKey, value); // for calib conf
    setDacMain(_currentKey, value); // actual output
  }
}

void handleDPot(float* params, int count) {
  if (count >= 1) {
    int value = (int)params[0];
    setGain(_currentKey, value); // for calib conf
    setDPot(_currentKey, value); // actual output
    _println("DPot changed to %d", value);
  }
}

void handleThreshold(float* params, int count) {
  if (count >= 1) {
    setThreshold(params[0]);
    _println("Threshold now %f V", params[0]);
  }
}

void handleUpdateLEDs(float* params, int count) {
  updateAllLEDs();
}

void handleLoadConfig(float* params, int count) {
  if (count > 0) _program = (int)params[0];
  loadConfigurationCSV();
  updateAllLEDs();
}

void handleSaveConfig(float* params, int count) {
  if (count > 0) _program = (int)params[0];
  saveConfigurationCSV();
}

void handleLoadCalib(float* params, int count) {
  loadCalibrationCSV();
}

void handleSaveCalib(float* params, int count) {
  saveCalibrationCSV();
}

void handleSetupLEDs(float* params, int count) {
  if (count == 0) {
    setupLEDs(NUM_KEYS);
    Serial.println("All LEDs were setup.");
  }
  if (count == 1) {
    int value = (int)params[0];
    setupLEDs(value);
    _println("%d LEDs were setup.", value);
  }  
}


void handleSetColorSelected(float* params, int count) {
  _println("%d parameters.", count);
  if (count == 0) {
    for (int k=_fromKey; k <= _toKey; k++) 
      setColor(k, 0, 0, 0);
    updateLEDs(_fromKey, _toKey);
    _println("Updated.");
  }
  if (count >= 3) {
    for (int k=_fromKey; k <= _toKey; k++) 
      setColor(k, (int)params[0], (int)params[1], (int)params[2]);
    updateLEDs(_fromKey, _toKey);
  }
}

void handleSetColorAll(float* params, int count) {
  if (count == 0) {
    setColorAll(0, 0, 0);
    updateAllLEDs();
  }
  if (count >= 3) {
    setColorAll((int)params[0], (int)params[1], (int)params[2]);
    updateAllLEDs();
  }
}

void handleMux(float* params, int count) {
  if (count == 1) {
    setMux(0, (int)params[0]);
  }
  if (count == 2) {
    setMux((int)params[0], (int)params[1]);
  }
}

void handleHelp(float* params, int count) {
  Serial.println("Available Commands:");
  for (int i = 0; i < commandCount; i++) {
    // Only print one alias per command group (optional: filter duplicates)
    Serial.print("  ");
    Serial.print(commandTable[i].name);
    Serial.print(" - ");
    Serial.println(commandTable[i].description);
  }
}

