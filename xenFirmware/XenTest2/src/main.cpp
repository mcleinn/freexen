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

// Forward decls for commands referenced by commandTable.
void handleDumpCalib(float* params, int count);
void handleDumpCalibMeta(float* params, int count);
void handleCalIssues(float* params, int count);
void handleAutoTuneIdle(float* params, int count);
void handleAutoTuneDump(float* params, int count);
void handleBootDrift(float* params, int count);
void handleDriftReset(float* params, int count);

int _outputFormat = 0; // 0=human, 1=jsonl
bool _diagActive = false;

// Bump this on every firmware change that touches serial protocol or behavior.
static const int XEN_FW_VERSION = 70;

static inline void mcpAck()
{
  // mcp2serial v0.1.0 only reads whatever is already available ~100ms after
  // sending. Long-running commands must emit an immediate line so the MCP
  // tool call doesn't look like a timeout.
  Serial.println("OK");
}

static inline void diagBegin()
{
  _diagActive = true;
  // Stop any lingering LED state and make tests own LEDs.
  setColorAll(0, 0, 0);
  updateAllLEDs();
}

static inline void diagEnd()
{
  _diagActive = false;
  // Return LEDs to configured state.
  updateAllLEDs();
}

static void printJsonKV(const char* k, const char* v, bool last=false) {
  Serial.print("\""); Serial.print(k); Serial.print("\":\""); Serial.print(v); Serial.print("\"");
  if (!last) Serial.print(",");
}

static void printJsonKV(const char* k, int v, bool last=false) {
  Serial.print("\""); Serial.print(k); Serial.print("\":"); Serial.print(v);
  if (!last) Serial.print(",");
}

static void printJsonKV(const char* k, float v, bool last=false) {
  Serial.print("\""); Serial.print(k); Serial.print("\":"); Serial.print(v, 6);
  if (!last) Serial.print(",");
}

static void beginJson(const char* type) {
  Serial.print("{");
  printJsonKV("type", type);
}

static void endJson() {
  Serial.println("}");
}

static void setAveragingRamOnly(int avg)
{
  _measureAvgStandard = avg;
  analogReadAveraging(avg);
}

static void setMuxDelayRamOnly(int us)
{
  if (us < 0) us = 0;
  if (us > 500) us = 500;
  _us_delay_after_mux = us;
}

// --- Autotune run capture (RAM only; keep one run) ---
static bool _autoHasRun = false;
static bool _autoRunComplete = false;
static uint32_t _autoRunId = 0;
static uint32_t _autoRunStartMs = 0;
static int _autoFromKey = 0;
static int _autoToKey = 0;
static int _autoOrigAvg = 0;
static int _autoOrigSd = 0;
static int _autoBestAvg = 0;
static int _autoBestSd = 0;
static int _autoTuneSeconds = 0;
static int _autoValidateSeconds = 0;
static int _autoBestTuneTotal = 0;
static int _autoBestValidateTotal = 0;
static int _autoTrialsRun = 0;
static int _autoMaxThrKey = -1;
static float _autoMaxThr = 0.0f;

// Metrics captured at end of autotune (RAM-only)
static float _autoKeysPerSec = 0.0f;
static float _autoRevisitMsEst = 0.0f;
static float _autoOverMin = 0.0f;
static float _autoOverMax = 0.0f;
static int _autoOverMinKey = -1;
static int _autoOverMaxKey = -1;

// Capture per-key info from the best candidate.
static uint16_t _autoCountsBest[NUM_KEYS];
static float _autoMaxExcursion[NUM_KEYS];
static float _autoThrBefore[NUM_KEYS];
static float _autoThrAfter[NUM_KEYS];

// Spike diagnostics for strict offenders (top N)
static const int AUTO_SPIKE_MAX = 12;
static int _autoSpikeN = 0;
static int _autoSpikeKey[AUTO_SPIKE_MAX];
static uint16_t _autoSpikeCountStrict[AUTO_SPIKE_MAX];
static float _autoSpikeMaxExcStrict[AUTO_SPIKE_MAX];
static float _autoSpikeThrAtStrict[AUTO_SPIKE_MAX];
static uint16_t _autoSpikeCount1[AUTO_SPIKE_MAX];
static uint16_t _autoSpikeCount2[AUTO_SPIKE_MAX];
static float _autoSpikeMaxExc1[AUTO_SPIKE_MAX];
static float _autoSpikeMaxExc2[AUTO_SPIKE_MAX];
static int _autoSpikeCarryoverLikely[AUTO_SPIKE_MAX];

// Capture trial summary (small, non-spam).
struct AutoTrial {
  int avg;
  int sd;
  int seconds;
  int total;
  int adjustedKeys;
  float maxThr;
};

static const int AUTO_MAX_TRIALS = 32;
static AutoTrial _autoTrials[AUTO_MAX_TRIALS];
static int _autoTrialsN = 0;

static int idleAuditCountsAndMax(int seconds, uint16_t* outCounts, float* outMaxExc)
{
  const uint32_t ms = (uint32_t)max(1, seconds) * 1000UL;
  memset(outCounts, 0, sizeof(uint16_t) * NUM_KEYS);
  for (int k = 0; k < NUM_KEYS; ++k) outMaxExc[k] = 0.0f;

  const int OFFSET = BOARDS_PER_ADC_CHANNEL * NUM_KEYS_PER_BOARD;
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    for (int muxPos = 0; muxPos < OFFSET; ++muxPos) {
      const int keyA = muxPos;
      const int keyB = muxPos + OFFSET;

      const bool doA = (keyA >= _fromKey) && (keyA <= _toKey) && _hasZero[keyA];
      const bool doB = (keyB >= _fromKey) && (keyB <= _toKey) && _hasZero[keyB];
      if (!doA && !doB) continue;

      setKey(muxPos);

      (void)getAdcUnscaledRawByIndex(ADC_UNSCALED1);
      (void)getAdcUnscaledRawByIndex(ADC_UNSCALED2);

      if (doA) {
        const float v = getAdcVoltage(getAdcUnscaled(keyA));
        const float swing = fabsf(v - _zeroVoltage[keyA]);
        if (swing > outMaxExc[keyA]) outMaxExc[keyA] = swing;
        if (swing > getEffectiveThresholdDelta(keyA) && outCounts[keyA] != 0xFFFF) outCounts[keyA]++;
      }
      if (doB) {
        const float v = getAdcVoltage(getAdcUnscaled(keyB));
        const float swing = fabsf(v - _zeroVoltage[keyB]);
        if (swing > outMaxExc[keyB]) outMaxExc[keyB] = swing;
        if (swing > getEffectiveThresholdDelta(keyB) && outCounts[keyB] != 0xFFFF) outCounts[keyB]++;
      }
    }
  }

  int total = 0;
  for (int k = _fromKey; k <= _toKey; ++k) total += outCounts[k];
  return total;
}

static void computeDynamicsMetrics(float &outOverMin, float &outOverMax, int &outOverMinKey, int &outOverMaxKey)
{
  outOverMin = 1e9f;
  outOverMax = -1e9f;
  outOverMinKey = -1;
  outOverMaxKey = -1;
  for (int k = _fromKey; k <= _toKey; ++k) {
    if (!_hasZero[k]) continue;
    const float over = _maxSwing[k] - _threshold_delta[k];
    if (over < outOverMin) { outOverMin = over; outOverMinKey = k; }
    if (over > outOverMax) { outOverMax = over; outOverMaxKey = k; }
  }
  if (outOverMinKey < 0) { outOverMin = 0.0f; outOverMax = 0.0f; }
}

static void spikeDiagStrictOffenders(int seconds)
{
  _autoSpikeN = 0;

  // Build top offender list from last strict counts stored in _autoCountsBest.
  for (int k = _autoFromKey; k <= _autoToKey; ++k) {
    if (!_hasZero[k]) continue;
    const uint16_t c = _autoCountsBest[k];
    if (c == 0) continue;
    // Insert into top list
    int ins = -1;
    for (int i = 0; i < _autoSpikeN; ++i) {
      if (c > _autoSpikeCountStrict[i]) { ins = i; break; }
    }
    if (ins < 0) {
      if (_autoSpikeN < AUTO_SPIKE_MAX) ins = _autoSpikeN;
      else continue;
    }
    if (_autoSpikeN < AUTO_SPIKE_MAX) {
      if (ins < _autoSpikeN) {
        for (int j = min(_autoSpikeN, AUTO_SPIKE_MAX - 1); j > ins; --j) {
          _autoSpikeKey[j] = _autoSpikeKey[j-1];
          _autoSpikeCountStrict[j] = _autoSpikeCountStrict[j-1];
        }
      }
      if (_autoSpikeN < AUTO_SPIKE_MAX) _autoSpikeN = min(_autoSpikeN + 1, AUTO_SPIKE_MAX);
      _autoSpikeKey[ins] = k;
      _autoSpikeCountStrict[ins] = c;
    }
  }

  // Copy strict pass max excursion/threshold for these keys (if available).
  for (int i = 0; i < _autoSpikeN; ++i) {
    const int key = _autoSpikeKey[i];
    _autoSpikeThrAtStrict[i] = _threshold_delta[key];
    _autoSpikeMaxExcStrict[i] = _autoMaxExcursion[key];
  }

  if (_autoSpikeN == 0) return;

  // Measure per-key one-sample vs two-sample behavior.
  for (int i = 0; i < _autoSpikeN; ++i) {
    _autoSpikeCount1[i] = 0;
    _autoSpikeCount2[i] = 0;
    _autoSpikeMaxExc1[i] = 0.0f;
    _autoSpikeMaxExc2[i] = 0.0f;
    _autoSpikeCarryoverLikely[i] = 0;
  }

  const int OFFSET = BOARDS_PER_ADC_CHANNEL * NUM_KEYS_PER_BOARD;
  const uint32_t ms = (uint32_t)max(1, seconds) * 1000UL;
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    for (int muxPos = 0; muxPos < OFFSET; ++muxPos) {
      setKey(muxPos);
      (void)getAdcUnscaledRawByIndex(ADC_UNSCALED1);
      (void)getAdcUnscaledRawByIndex(ADC_UNSCALED2);

      // Sample1 then sample2 (same mux pos)
      for (int i = 0; i < _autoSpikeN; ++i) {
        const int key = _autoSpikeKey[i];
        if (key != muxPos && key != muxPos + OFFSET) continue;
        const float thr = _threshold_delta[key];
        const float zero = _zeroVoltage[key];

        const float v1 = getAdcVoltage(getAdcUnscaled(key));
        const float exc1 = fabsf(v1 - zero);
        if (exc1 > _autoSpikeMaxExc1[i]) _autoSpikeMaxExc1[i] = exc1;
        if (exc1 > thr && _autoSpikeCount1[i] != 0xFFFF) _autoSpikeCount1[i]++;

        const float v2 = getAdcVoltage(getAdcUnscaled(key));
        const float exc2 = fabsf(v2 - zero);
        if (exc2 > _autoSpikeMaxExc2[i]) _autoSpikeMaxExc2[i] = exc2;
        if (exc2 > thr && _autoSpikeCount2[i] != 0xFFFF) _autoSpikeCount2[i]++;
      }
    }
  }

  for (int i = 0; i < _autoSpikeN; ++i) {
    const float delta = _autoSpikeMaxExc1[i] - _autoSpikeMaxExc2[i];
    if (_autoSpikeCount1[i] > 0 && _autoSpikeCount2[i] == 0) _autoSpikeCarryoverLikely[i] = 1;
    else if (delta > 0.010f && _autoSpikeCount1[i] > _autoSpikeCount2[i]) _autoSpikeCarryoverLikely[i] = 1;
  }
}

static int idleAuditCounts(int seconds, uint16_t* outCounts)
{
  static float dummyMax[NUM_KEYS];
  return idleAuditCountsAndMax(seconds, outCounts, dummyMax);
}

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
  {"a", handleAveraging, "Set averaging value to 1,2,4,8,16,32"},
  {"sd", handleMuxDelay, "Set mux settle delay in us"},
  {"thr", handleThreshold, "Set threshold"},
  {"fmt", handleFormat, "Set output format (0=human, 1=jsonl)"},
  {"c", handleSetColorSelected, "Set color for selected keys (three parameters 0-255)"},
  {"ca", handleSetColorAll, "Set color for all keys (three parameters 0-255)"},
  {"x", handleCalibrate, "Enter calibration mode"},
  {"r", handleRun, "Enter run mode"},
  {"p", handlePlot, "Enter plot mode"},
  {"n", handleMeasureNoise, "Measure noise for selected keys (duration in s)"},
  {"ccalib", handleClearCalib, "Clear calibration for selected keys"},
  {"rk", handleReadKey, "Read one key snapshot (optional: key)"},
  {"rs", handleReadStats, "Read key stats (key,ms)"},
  {"pr", handleProblemReport, "Noise/problem report for selected keys (seconds)"},
  {"rate", handleRate, "Scan rate estimate (ms)"},
  {"idle", handleIdleAudit, "Runtime idle false-trigger audit (seconds)"},
  {"rperf", handleRuntimePerf, "Runtime scan performance (seconds)"},
  {"st", handleSettleTest, "Settling test (keyA,keyB,delay_us,samples)"},
  {"sts", handleSettleSweep, "Settling sweep (keyA,keyB)"},
  {"testa", handleTest0, "Test A: self check"},
  {"testb", handleTest1, "Test B: sentinel press"},
  {"testc", handleTest2, "Test C: settling"},
  {"testd", handleTest3, "Test D: noise survey (seconds)"},
  {"teste", handleTest6, "Test E: perf sweep"},
  {"rst", handleReset, "Software reset"},
  {"ver", handleVersion, "Print firmware version"},
  {"calstat", handleCalStat, "Calibration status (all keys)"},
  {"dcalib", handleDumpCalib, "Dump calib.csv as JSONL"},
  {"dmeta", handleDumpCalibMeta, "Dump calib_meta.csv as JSONL"},
  {"calissues", handleCalIssues, "Print last calibration issues (RAM)"},
  {"autotune", handleAutoTuneIdle, "Auto-tune avg/sd for idle=0 (RAM only)"},
  {"autodump", handleAutoTuneDump, "Dump last autotune run (JSONL)"},
  {"bootdrift", handleBootDrift, "Print boot drift summary (cached)"},
  {"driftreset", handleDriftReset, "Re-measure boot drift and apply compensation"},
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

  bool connected = Serial && Serial.dtr();  
  if (connected) {
    _debugMode = true;
    _bootedInDebugMode = true;
    Serial.println("Debug mode enabled.");
    _mainLoopState = 0;
    _program = 9;
  } else {
    _mainLoopState = 3;
    _program = 0;
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

  // Calibration is loaded by setupCalibration() for both debug + non-debug.

  // Boot status banner
  if (Serial && Serial.dtr()) {
    if (_outputFormat) {
      beginJson("boot");
      printJsonKV("fw", XEN_FW_VERSION);
      printJsonKV("bootDebug", _bootedInDebugMode ? 1 : 0);
      printJsonKV("calibAutoLoadOk", _calibAutoLoadOk ? 1 : 0, true);
      endJson();
    } else {
      _println("boot fw=%d bootDebug=%d calibAutoLoadOk=%d", XEN_FW_VERSION, _bootedInDebugMode ? 1 : 0, _calibAutoLoadOk ? 1 : 0);
    }
  }

  Serial.println("Ready to receive commands (e.g. a123)...");
}

void loop() {
  if (_debugMode)
    checkSerial();

  // Diagnostics/tests take over the device (LEDs, timing). Do not run normal scanning.
  if (_diagActive) {
    return;
  }

  if (_mainLoopState == 1) {
    // Plotter mode
    _print("Min:0.0,");
    for (int k=_fromKey; k<= _toKey; k++) {
      setKey(k);
      int adcUnscaled1 = getAdcUnscaled(k);
      float unscaledVoltage1 = getAdcVoltage(adcUnscaled1);

      _print("U%d:%f,", k+1, unscaledVoltage1);
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
    scanKeysNormal();
  } else if (_mainLoopState == 4) {
    // Noise operation mode
    scanNoise(500);
    showNoise(0);
  }

  // In calibration mode, LEDs are driven explicitly by calibration steps.
  // Avoid re-showing stale pixels which can look like flicker on unrelated keys.
  if (_mainLoopState != 2) {
    LEDStrip->show();
  }
  updateDebugState();
}

void updateDebugState()
{
    bool connected = Serial && Serial.dtr();  

    if (connected && !_debugMode) {            
      _debugMode      = true;

      // When debug is enabled (often after connecting later), suppress any
      // running loop modes that spam serial (e.g. calibration/plot/noise).
      // The user can explicitly re-enter a loop mode via commands.
      if (_mainLoopState != 2) {
        _mainLoopState = 0;
      }

      if (_outputFormat) {
        beginJson("debug");
        printJsonKV("enabled", 1);
        printJsonKV("bootDebug", _bootedInDebugMode ? 1 : 0);
        printJsonKV("enteredLater", _bootedInDebugMode ? 0 : 1);
        printJsonKV("fw", XEN_FW_VERSION);
        printJsonKV("calibAutoLoadOk", _calibAutoLoadOk ? 1 : 0, true);
        endJson();
      } else {
        Serial.println("Debug mode enabled.");
        _println("debug enteredLater=%d fw=%d calibAutoLoadOk=%d",
                _bootedInDebugMode ? 0 : 1,
                XEN_FW_VERSION,
                _calibAutoLoadOk ? 1 : 0);
      }

      // Print minimal drift summary on debug attach (no re-measure).
      // Baseline compensation already happened in setupCalibration().
      extern void bootDriftPrintSummary();
      bootDriftPrintSummary();
    }
    else if (!connected && _debugMode) {        
      _debugMode      = false;
        // no Serial.print() here – there’s nowhere to send it
    }
}

void checkSerial() {
  static char inputBuffer[INPUT_BUFFER_SIZE];
  static int inputIndex = 0;
  
  while (Serial.available() > 0) {
    char incomingByte = (char)Serial.read();

    // mcp2serial and many terminals send CRLF. Ignore CR to avoid
    // corrupting the next command in the buffer.
    if (incomingByte == '\r') {
      continue;
    }
    
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
    _println("Cancelling calibration.");
    return;
  }
  _mainLoopState = 2;
  _manualCalibration = true;
  _calibLoopState = STATE_CALIBRATION_START;
  _println("Entered calibration loop.");
}

void handleRun(float* params, int count) {
  if (_mainLoopState == 3) {
    _mainLoopState = 0;
    _println("Cancelling run.");
    return;
  }
  _mainLoopState = 3;
  _println("Entered run loop.");
}

void handlePlot(float* params, int count) {
  if (_mainLoopState == 1) {
    _mainLoopState = 0;
    _println("Cancelling plot.");
    return;
  }
  _mainLoopState = 1;
  _println("Entered plot loop.");
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

void handleThreshold(float* params, int count) {
  if (count >= 1) {
    for (int k=_fromKey; k <= _toKey; k++) 
      setThreshold(k, params[0]);
    _println("Threshold for %d to %d now %f V", _fromKey+1, _toKey+1, params[0]);
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

// Boot drift summary (cached) and reset (remeasure+apply).
extern void bootDriftPrintSummary();
extern void driftReset();

void handleBootDrift(float* params, int count)
{
  (void)params; (void)count;
  mcpAck();
  bootDriftPrintSummary();
}

void handleDriftReset(float* params, int count)
{
  (void)params; (void)count;
  mcpAck();
  driftReset();
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

void handleAveraging(float* params, int count) {
  if (count == 1) {
    _measureAvgStandard = (int)params[0];
    // Hardware averaging caps at 32 on Teensy 4.x core. For avg>32 we keep
    // hardware at 32 and do the rest in software (see scanKey()).
    analogReadAveraging(_measureAvgStandard > 32 ? 32 : _measureAvgStandard);
    _println("ADC set to averaging %d.", _measureAvgStandard);
  }
}

void handleMeasureNoise(float* params, int count) {
  _mainLoopState = 0;
  if (count == 0) 
    scanNoise(1000);
  else if (count >= 1)
    scanNoise((int)(params[0]*1000));

  showNoise(0);

  printNoiseLevels();
}

void handleFormat(float* params, int count)
{
  if (count >= 1) {
    _outputFormat = ((int)params[0]) ? 1 : 0;
  }

  if (_outputFormat) {
    beginJson("fmt");
    printJsonKV("value", _outputFormat, true);
    endJson();
  } else {
    _println("Format set to %s", _outputFormat ? "jsonl" : "human");
  }
}

void handleMuxDelay(float* params, int count)
{
  if (count >= 1) {
    _us_delay_after_mux = max(0, (int)params[0]);
  }

  if (_outputFormat) {
    beginJson("sd");
    printJsonKV("us", _us_delay_after_mux, true);
    endJson();
  } else {
    _println("Mux settle delay now %d us", _us_delay_after_mux);
  }
}

void handleReset(float* params, int count)
{
  (void)params; (void)count;
  if (_outputFormat) {
    beginJson("rst");
    printJsonKV("ok", 1, true);
    endJson();
  } else {
    _println("Resetting...");
  }
  delay(100);
  SCB_AIRCR = 0x05FA0004; // system reset request
  while (1) {}
}

void handleVersion(float* params, int count)
{
  (void)params; (void)count;
  mcpAck();
  if (_outputFormat) {
    beginJson("ver");
    printJsonKV("fw", XEN_FW_VERSION, true);
    endJson();
  } else {
    _println("fw=%d", XEN_FW_VERSION);
  }
}

void handleCalStat(float* params, int count)
{
  (void)params; (void)count;
  mcpAck();

  int hasZero = 0;
  int hasThr = 0;
  float maxSigma = 0.0f;
  float minSigma = 1e9f;
  int nSigma = 0;

  for (int k = 0; k < NUM_KEYS; ++k) {
    if (_hasZero[k]) hasZero++;
    if (_threshold_delta[k] > 1e-6f) hasThr++;
    const float s = _noiseUnscaled[k];
    if (s > 0.0f) {
      nSigma++;
      if (s > maxSigma) maxSigma = s;
      if (s < minSigma) minSigma = s;
    }
  }

  if (_outputFormat) {
    beginJson("calstat");
    printJsonKV("total", NUM_KEYS);
    printJsonKV("hasZero", hasZero);
    printJsonKV("hasThr", hasThr);
    printJsonKV("nSigma", nSigma);
    printJsonKV("minSigma", (nSigma ? minSigma : 0.0f));
    printJsonKV("maxSigma", maxSigma, true);
    endJson();
  } else {
    _println("calstat total=%d hasZero=%d hasThr=%d nSigma=%d minSigma=%.6f maxSigma=%.6f",
             NUM_KEYS, hasZero, hasThr, nSigma, (nSigma ? minSigma : 0.0f), maxSigma);
  }
}

static void jsonPrintString(const char* s)
{
  // Minimal JSON string escaping for ASCII-friendly content.
  Serial.print('"');
  for (const char* p = s; *p; ++p) {
    const char c = *p;
    if (c == '"' || c == '\\') {
      Serial.print('\\');
      Serial.print(c);
    } else if (c == '\n') {
      Serial.print("\\n");
    } else if (c == '\r') {
      Serial.print("\\r");
    } else if (c == '\t') {
      Serial.print("\\t");
    } else {
      Serial.print(c);
    }
  }
  Serial.print('"');
}

void handleDumpCalib(float* params, int count)
{
  (void)params;
  (void)count;
  mcpAck();

  // Report whether current RAM calibration differs from last loaded/saved state.
  if (_outputFormat) {
    beginJson("dcalib_start");
    printJsonKV("source", "ram");
    printJsonKV("dirty", _calibDirty ? 1 : 0, true);
    endJson();
  } else {
    _println("dcalib source=ram dirty=%d", _calibDirty ? 1 : 0);
  }

  for (int key = 0; key < NUM_KEYS; ++key) {
    int board = 0, boardKey = 0;
    getBoardAndBoardKey(key, board, boardKey);
    Serial.print("{\"type\":\"dcalib_row\",\"key\":");
    Serial.print(key);
    Serial.print(",\"board\":");
    Serial.print(board);
    Serial.print(",\"boardKey\":");
    Serial.print(boardKey);
    Serial.print(",\"hasZero\":");
    Serial.print(_hasZero[key] ? 1 : 0);
    Serial.print(",\"maxSwing\":");
    Serial.print(_maxSwing[key], 6);
    Serial.print(",\"pol\":");
    Serial.print((int)_polarization[key]);
    Serial.print(",\"sigma\":");
    Serial.print(_noiseUnscaled[key], 6);
    Serial.print(",\"thr\":");
    Serial.print(_threshold_delta[key], 6);
    Serial.print(",\"zero\":");
    Serial.print(_zeroVoltage[key], 6);
    Serial.println("}");
  }

  Serial.println("{\"type\":\"dcalib_done\",\"ok\":1}");
}

void handleCalIssues(float* params, int count)
{
  (void)params;
  (void)count;
  mcpAck();
  printLastCalibrationIssues();
}

void handleDumpCalibMeta(float* params, int count)
{
  (void)params;
  (void)count;
  mcpAck();

  File f = SD.open("calib_meta.csv", FILE_READ);
  if (!f) {
    if (_outputFormat) {
      Serial.println("{\"type\":\"dmeta\",\"ok\":0,\"error\":\"open_failed\"}");
    } else {
      _println("dmeta error: failed to open calib_meta.csv");
    }
    return;
  }

  if (_outputFormat) {
    Serial.println("{\"type\":\"dmeta_start\",\"file\":\"calib_meta.csv\"}");
  }

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    if (line.startsWith("#")) continue;

    int lastPos = 0;
    int nextPos = line.indexOf(',', lastPos);
    if (nextPos < 0) continue;
    const int key = line.substring(lastPos, nextPos).toInt();
    lastPos = nextPos + 1;

    nextPos = line.indexOf(',', lastPos);
    if (nextPos < 0) continue;
    const int board = line.substring(lastPos, nextPos).toInt();
    lastPos = nextPos + 1;

    nextPos = line.indexOf(',', lastPos);
    if (nextPos < 0) continue;
    const int boardKey = line.substring(lastPos, nextPos).toInt();
    lastPos = nextPos + 1;

    nextPos = line.indexOf(',', lastPos);
    if (nextPos < 0) continue;
    const int skipped = line.substring(lastPos, nextPos).toInt();
    lastPos = nextPos + 1;

    nextPos = line.indexOf(',', lastPos);
    if (nextPos < 0) continue;
    const int notTrig = line.substring(lastPos, nextPos).toInt();
    lastPos = nextPos + 1;

    nextPos = line.indexOf(',', lastPos);
    if (nextPos < 0) continue;
    const int selfTrig = line.substring(lastPos, nextPos).toInt();
    lastPos = nextPos + 1;

    nextPos = line.indexOf(',', lastPos);
    if (nextPos < 0) continue;
    const int notRel = line.substring(lastPos, nextPos).toInt();
    lastPos = nextPos + 1;

    nextPos = line.indexOf(',', lastPos);
    if (nextPos < 0) continue;
    const int slightly = line.substring(lastPos, nextPos).toInt();
    lastPos = nextPos + 1;

    const unsigned long releaseMs = (unsigned long)line.substring(lastPos).toInt();

    if (_outputFormat) {
      Serial.print("{\"type\":\"dmeta_row\",\"key\":");
      Serial.print(key);
      Serial.print(",\"board\":");
      Serial.print(board);
      Serial.print(",\"boardKey\":");
      Serial.print(boardKey);
      Serial.print(",\"skipped\":");
      Serial.print(skipped);
      Serial.print(",\"notTriggering\":");
      Serial.print(notTrig);
      Serial.print(",\"selfTriggering\":");
      Serial.print(selfTrig);
      Serial.print(",\"notReleasing\":");
      Serial.print(notRel);
      Serial.print(",\"slightlyStuck\":");
      Serial.print(slightly);
      Serial.print(",\"releaseMs\":");
      Serial.print(releaseMs);
      Serial.println("}");
    } else {
      _println("meta key=%d board=%d boardKey=%d skip=%d notTrig=%d selfTrig=%d notRel=%d slightly=%d releaseMs=%lu",
              key + 1, board + 1, boardKey + 1,
              skipped, notTrig, selfTrig, notRel, slightly, releaseMs);
    }
  }

  f.close();
  if (_outputFormat) {
    Serial.println("{\"type\":\"dmeta_done\",\"ok\":1}");
  }
}

void handleAutoTuneDump(float* params, int count)
{
  (void)params;
  (void)count;
  mcpAck();

  if (!_autoHasRun) {
    if (_outputFormat) {
      Serial.println("{\"type\":\"autodump\",\"ok\":0,\"error\":\"no_run\"}");
    } else {
      _println("autodump: no run");
    }
    return;
  }

  if (_outputFormat) {
    Serial.println("{\"type\":\"autodump_start\"}");
    beginJson("autodump");
    printJsonKV("ok", 1);
    printJsonKV("complete", _autoRunComplete ? 1 : 0);
    printJsonKV("runId", (int)_autoRunId);
    printJsonKV("from", _autoFromKey);
    printJsonKV("to", _autoToKey);
    printJsonKV("tuneSeconds", _autoTuneSeconds);
    printJsonKV("validateSeconds", _autoValidateSeconds);
    printJsonKV("origAvg", _autoOrigAvg);
    printJsonKV("origSd_us", _autoOrigSd);
    printJsonKV("bestAvg", _autoBestAvg);
    printJsonKV("bestSd_us", _autoBestSd);
    printJsonKV("bestTuneTotal", _autoBestTuneTotal);
    printJsonKV("bestValidateTotal", _autoBestValidateTotal);
    printJsonKV("trialsRun", _autoTrialsRun);
    printJsonKV("trialsLogged", _autoTrialsN);
    printJsonKV("maxThr", _autoMaxThr);
    printJsonKV("maxThrKey", _autoMaxThrKey, true);
    endJson();

    beginJson("autodump_rate");
    printJsonKV("avg", _autoBestAvg);
    printJsonKV("sd_us", _autoBestSd);
    printJsonKV("keysPerSec", _autoKeysPerSec);
    printJsonKV("revisitMsEst", _autoRevisitMsEst, true);
    endJson();

    beginJson("autodump_dyn");
    printJsonKV("overMin", _autoOverMin);
    printJsonKV("overMinKey", _autoOverMinKey);
    printJsonKV("overMax", _autoOverMax);
    printJsonKV("overMaxKey", _autoOverMaxKey, true);
    endJson();

    for (int i = 0; i < _autoTrialsN; ++i) {
      beginJson("autodump_trial");
      printJsonKV("i", i + 1);
      printJsonKV("avg", _autoTrials[i].avg);
      printJsonKV("sd_us", _autoTrials[i].sd);
      printJsonKV("seconds", _autoTrials[i].seconds);
      printJsonKV("total", _autoTrials[i].total);
      printJsonKV("adjustedKeys", _autoTrials[i].adjustedKeys);
      printJsonKV("maxThr", _autoTrials[i].maxThr, true);
      endJson();
    }

    // (rate/dynamics rows emitted via dedicated variables; no synthetic trials)

    // Per-key details for best candidate: only emit interesting keys to avoid spam.
    // Criteria: offenders during validate (counts>0) OR threshold raised OR large excursion.
    for (int k = _autoFromKey; k <= _autoToKey; ++k) {
      if (!_hasZero[k]) continue;
      const bool offender = (_autoCountsBest[k] > 0);
      const bool raised = (_autoThrAfter[k] > _autoThrBefore[k] + 1e-6f);
      const bool bigExc = (_autoMaxExcursion[k] > 0.05f);
      if (!offender && !raised && !bigExc) continue;
      beginJson("autodump_key");
      printJsonKV("key", k);
      printJsonKV("count", (int)_autoCountsBest[k]);
      printJsonKV("maxExc", _autoMaxExcursion[k]);
      printJsonKV("thrBefore", _autoThrBefore[k]);
      printJsonKV("thrAfter", _autoThrAfter[k], true);
      endJson();
    }

    Serial.println("{\"type\":\"autodump_done\",\"ok\":1}");
  } else {
    _println("autodump runId=%lu best avg=%d sd=%d tuneTotal=%d validateTotal=%d",
             (unsigned long)_autoRunId, _autoBestAvg, _autoBestSd, _autoBestTuneTotal, _autoBestValidateTotal);
  }
}

void handleClearCalib(float* params, int count)
{
  (void)params;
  (void)count;

  clearCalibration(_fromKey, _toKey);
  _println("Cleared calibration for keys %d to %d", _fromKey + 1, _toKey + 1);
}

void handleReadKey(float* params, int count)
{
  mcpAck();
  int key = _fromKey;
  if (count >= 1) key = (int)params[0] - 1;
  if (key < 0 || key >= NUM_KEYS) return;

  setKey(key);
  const int adc = getAdcUnscaled(key);
  const float v = getAdcVoltage(adc);

  // peakDetect internal state isn't exposed; provide basic derived values.
  const float zero = _zeroVoltage[key];
  const float thr = _threshold_delta[key];
  const float swing = _hasZero[key] ? fabsf(v - zero) : 0.0f;

  int board, boardKey;
  getBoardAndBoardKey(key, board, boardKey);

  if (_outputFormat) {
    beginJson("rk");
    printJsonKV("key", key);
    printJsonKV("board", board);
    printJsonKV("boardKey", boardKey);
    printJsonKV("adc", adc);
    printJsonKV("v", v);
    printJsonKV("hasZero", _hasZero[key] ? 1 : 0);
    printJsonKV("zero", zero);
    printJsonKV("swing", swing);
    printJsonKV("thr", thr);
    printJsonKV("maxSwing", _maxSwing[key]);
    printJsonKV("pol", (int)_polarization[key], true);
    endJson();
  } else {
    _println("rk key=%d board=%d boardKey=%d adc=%d v=%.4f zero=%s%.4f swing=%.4f thr=%.4f maxSwing=%.4f pol=%d",
             key + 1, board, boardKey, adc, v,
             _hasZero[key] ? "" : "(unset)", zero,
             swing, thr, _maxSwing[key], (int)_polarization[key]);
  }
}

void handleReadStats(float* params, int count)
{
  mcpAck();
  if (count < 1) return;
  const int key = (int)params[0] - 1;
  const int ms = (count >= 2) ? (int)params[1] : 500;
  if (key < 0 || key >= NUM_KEYS) return;

  setKey(key);
  const uint32_t t0 = millis();
  uint32_t n = 0;
  float mean = 0.0f;
  float m2 = 0.0f;
  float vMin = 1e9f;
  float vMax = -1e9f;
  while (millis() - t0 < (uint32_t)ms) {
    const float v = getAdcVoltage(getAdcUnscaled(key));
    if (v < vMin) vMin = v;
    if (v > vMax) vMax = v;
    ++n;
    const float delta = v - mean;
    mean += delta / n;
    m2 += delta * (v - mean);
  }
  const float sigma = (n > 1) ? sqrtf(m2 / (n - 1)) : 0.0f;
  int board, boardKey;
  getBoardAndBoardKey(key, board, boardKey);

  if (_outputFormat) {
    beginJson("rs");
    printJsonKV("key", key);
    printJsonKV("board", board);
    printJsonKV("boardKey", boardKey);
    printJsonKV("ms", ms);
    printJsonKV("n", (int)n);
    printJsonKV("min", vMin);
    printJsonKV("mean", mean);
    printJsonKV("max", vMax);
    printJsonKV("sigma", sigma, true);
    endJson();
  } else {
    _println("rs key=%d n=%lu min=%.4f mean=%.4f max=%.4f sigma=%.6f",
             key + 1, (unsigned long)n, vMin, mean, vMax, sigma);
  }
}

void handleProblemReport(float* params, int count)
{
  mcpAck();
  diagBegin();
  const int seconds = (count >= 1) ? max(1, (int)params[0]) : 2;
  const int ms = seconds * 1000;
  scanNoise(ms);

  const float thresholdMin = 0.03f;
  const float kSigma = 8.0f;

  if (_outputFormat) {
    beginJson("pr");
    printJsonKV("from", _fromKey);
    printJsonKV("to", _toKey);
    printJsonKV("seconds", seconds, true);
    endJson();
  } else {
    _println("pr seconds=%d range=%d..%d", seconds, _fromKey + 1, _toKey + 1);
  }

  // Stream rows (JSONL) or CSV
  if (_outputFormat) {
    for (int k = _fromKey; k <= _toKey; ++k) {
      const float sigma = _noiseUnscaled[k];
      const float thrSuggest = max(thresholdMin, kSigma * sigma);
      beginJson("pr_row");
      printJsonKV("key", k);
      printJsonKV("sigma", sigma);
      printJsonKV("thr", _threshold_delta[k]);
      printJsonKV("thrSuggest", thrSuggest);
      printJsonKV("zero", _zeroVoltage[k]);
      printJsonKV("hasZero", _hasZero[k] ? 1 : 0, true);
      endJson();
    }
  } else {
    Serial.println("key,sigmaV,thrV,thrSuggestV,zeroV,hasZero");
    for (int k = _fromKey; k <= _toKey; ++k) {
      const float sigma = _noiseUnscaled[k];
      const float thrSuggest = max(thresholdMin, kSigma * sigma);
      Serial.print(k + 1);
      Serial.print(",");
      Serial.print(sigma, 6);
      Serial.print(",");
      Serial.print(_threshold_delta[k], 6);
      Serial.print(",");
      Serial.print(thrSuggest, 6);
      Serial.print(",");
      Serial.print(_zeroVoltage[k], 6);
      Serial.print(",");
      Serial.println(_hasZero[k] ? 1 : 0);
    }
  }

  diagEnd();
}

void handleRate(float* params, int count)
{
  mcpAck();
  diagBegin();
  const int ms = (count >= 1) ? max(100, (int)params[0]) : 1000;
  const int OFFSET = BOARDS_PER_ADC_CHANNEL * NUM_KEYS_PER_BOARD;

  uint32_t steps = 0;
  uint32_t reads = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < (uint32_t)ms) {
    for (int muxPos = 0; muxPos < OFFSET; ++muxPos) {
      setKey(muxPos);
      // two reads (A/B banks) when in range, approximate by range coverage
      const int keyA = muxPos;
      const int keyB = muxPos + OFFSET;
      if (keyA >= _fromKey && keyA <= _toKey) { (void)getAdcUnscaled(keyA); ++reads; }
      if (keyB >= _fromKey && keyB <= _toKey) { (void)getAdcUnscaled(keyB); ++reads; }
      ++steps;
    }
  }
  const float secs = (float)ms / 1000.0f;
  const float stepsPerSec = steps / secs;
  const float readsPerSec = reads / secs;
  const float keysPerSec = readsPerSec; // one ADC read per key measurement

  if (_outputFormat) {
    beginJson("rate");
    printJsonKV("ms", ms);
    printJsonKV("muxStepsPerSec", stepsPerSec);
    printJsonKV("adcReadsPerSec", readsPerSec);
    printJsonKV("keysPerSec", keysPerSec, true);
    endJson();
  } else {
    _println("rate ms=%d muxSteps/s=%.1f adcReads/s=%.1f keys/s=%.1f", ms, stepsPerSec, readsPerSec, keysPerSec);
  }

  diagEnd();
}

static float keySteadyVoltage(int key)
{
  setKey(key);
  delayMicroseconds(max(0, _us_delay_after_mux));
  // take a small average of a few samples
  float sum = 0.0f;
  const int n = 8;
  for (int i = 0; i < n; ++i) {
    sum += getAdcVoltage(getAdcUnscaled(key));
  }
  return sum / (float)n;
}

void handleSettleTest(float* params, int count)
{
  mcpAck();
  diagBegin();
  if (count < 4) { diagEnd(); return; }
  const int keyA = (int)params[0] - 1;
  const int keyB = (int)params[1] - 1;
  const int delayUs = max(0, (int)params[2]);
  const int samples = max(1, (int)params[3]);
  if (keyA < 0 || keyA >= NUM_KEYS || keyB < 0 || keyB >= NUM_KEYS) return;

  const float steadyA = keySteadyVoltage(keyA);
  const float steadyB = keySteadyVoltage(keyB);

  auto measureDir = [&](int fromKey, int toKey, float steadyTo, float &meanErr, float &maxErr) {
    meanErr = 0.0f;
    maxErr = 0.0f;
    for (int i = 0; i < samples; ++i) {
      setKey(fromKey);
      delayMicroseconds(delayUs);
      setKey(toKey);
      delayMicroseconds(delayUs);
      const float v = getAdcVoltage(getAdcUnscaled(toKey));
      const float err = fabsf(v - steadyTo);
      meanErr += err;
      if (err > maxErr) maxErr = err;
    }
    meanErr /= (float)samples;
  };

  float meanAB, maxAB, meanBA, maxBA;
  measureDir(keyA, keyB, steadyB, meanAB, maxAB);
  measureDir(keyB, keyA, steadyA, meanBA, maxBA);

  if (_outputFormat) {
    beginJson("st");
    printJsonKV("keyA", keyA);
    printJsonKV("keyB", keyB);
    printJsonKV("delay_us", delayUs);
    printJsonKV("samples", samples);
    printJsonKV("steadyA", steadyA);
    printJsonKV("steadyB", steadyB);
    printJsonKV("meanErrAB", meanAB);
    printJsonKV("maxErrAB", maxAB);
    printJsonKV("meanErrBA", meanBA);
    printJsonKV("maxErrBA", maxBA, true);
    endJson();
  } else {
    _println("st A=%d B=%d delay_us=%d samples=%d steadyA=%.4f steadyB=%.4f meanErrAB=%.6f maxErrAB=%.6f meanErrBA=%.6f maxErrBA=%.6f",
             keyA + 1, keyB + 1, delayUs, samples, steadyA, steadyB, meanAB, maxAB, meanBA, maxBA);
  }

  diagEnd();
}

void handleSettleSweep(float* params, int count)
{
  mcpAck();
  diagBegin();
  if (count < 2) { diagEnd(); return; }
  const int keyA = (int)params[0] - 1;
  const int keyB = (int)params[1] - 1;
  if (keyA < 0 || keyA >= NUM_KEYS || keyB < 0 || keyB >= NUM_KEYS) return;

  const int delays[] = {0, 5, 10, 20, 50, 100, 200};
  const int samples = 50;

  if (_outputFormat) {
    beginJson("sts");
    printJsonKV("keyA", keyA);
    printJsonKV("keyB", keyB);
    printJsonKV("samples", samples, true);
    endJson();
  } else {
    _println("sts keyA=%d keyB=%d", keyA + 1, keyB + 1);
    Serial.println("delay_us,meanErrAB,maxErrAB,meanErrBA,maxErrBA");
  }

  float bestDelay = delays[0];
  float bestScore = 1e9f;

  for (int i = 0; i < (int)(sizeof(delays)/sizeof(delays[0])); ++i) {
    float meanAB, maxAB, meanBA, maxBA;
    float steadyA = keySteadyVoltage(keyA);
    float steadyB = keySteadyVoltage(keyB);
    // reuse settle test core
    float tmpMean, tmpMax;
    // A->B
    tmpMean = 0; tmpMax = 0;
    for (int s = 0; s < samples; ++s) {
      setKey(keyA);
      delayMicroseconds(delays[i]);
      setKey(keyB);
      delayMicroseconds(delays[i]);
      const float v = getAdcVoltage(getAdcUnscaled(keyB));
      const float err = fabsf(v - steadyB);
      tmpMean += err;
      if (err > tmpMax) tmpMax = err;
    }
    meanAB = tmpMean / (float)samples;
    maxAB = tmpMax;
    // B->A
    tmpMean = 0; tmpMax = 0;
    for (int s = 0; s < samples; ++s) {
      setKey(keyB);
      delayMicroseconds(delays[i]);
      setKey(keyA);
      delayMicroseconds(delays[i]);
      const float v = getAdcVoltage(getAdcUnscaled(keyA));
      const float err = fabsf(v - steadyA);
      tmpMean += err;
      if (err > tmpMax) tmpMax = err;
    }
    meanBA = tmpMean / (float)samples;
    maxBA = tmpMax;

    const float score = max(maxAB, maxBA);
    if (score < bestScore) { bestScore = score; bestDelay = (float)delays[i]; }

    if (_outputFormat) {
      beginJson("sts_row");
      printJsonKV("delay_us", delays[i]);
      printJsonKV("meanErrAB", meanAB);
      printJsonKV("maxErrAB", maxAB);
      printJsonKV("meanErrBA", meanBA);
      printJsonKV("maxErrBA", maxBA, true);
      endJson();
    } else {
      Serial.print(delays[i]); Serial.print(",");
      Serial.print(meanAB, 6); Serial.print(",");
      Serial.print(maxAB, 6); Serial.print(",");
      Serial.print(meanBA, 6); Serial.print(",");
      Serial.println(maxBA, 6);
    }
  }

  if (_outputFormat) {
    beginJson("sts_best");
    printJsonKV("delay_us", (int)bestDelay);
    printJsonKV("score", bestScore, true);
    endJson();
  } else {
    _println("sts best delay_us=%d score(maxErr)=%.6f", (int)bestDelay, bestScore);
  }

  diagEnd();
}

void handleIdleAudit(float* params, int count)
{
  mcpAck();
  diagBegin();

  const int seconds = (count >= 1) ? max(1, (int)params[0]) : 30;
  const uint32_t ms = (uint32_t)seconds * 1000UL;

  // Simple idle audit based on current calibration thresholds.
  // Keys without baseline are skipped.
  static uint16_t counts[NUM_KEYS];
  memset(counts, 0, sizeof(counts));

  const int OFFSET = BOARDS_PER_ADC_CHANNEL * NUM_KEYS_PER_BOARD;
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    for (int muxPos = 0; muxPos < OFFSET; ++muxPos) {
      const int keyA = muxPos;
      const int keyB = muxPos + OFFSET;

      const bool doA = (keyA >= _fromKey) && (keyA <= _toKey) && _hasZero[keyA];
      const bool doB = (keyB >= _fromKey) && (keyB <= _toKey) && _hasZero[keyB];
      if (!doA && !doB) continue;

      setKey(muxPos);

      (void)getAdcUnscaledRawByIndex(ADC_UNSCALED1);
      (void)getAdcUnscaledRawByIndex(ADC_UNSCALED2);

      if (doA) {
        const float v = getAdcVoltage(getAdcUnscaled(keyA));
        const float swing = fabsf(v - _zeroVoltage[keyA]);
        if (swing > getEffectiveThresholdDelta(keyA) && counts[keyA] != 0xFFFF) counts[keyA]++;
      }
      if (doB) {
        const float v = getAdcVoltage(getAdcUnscaled(keyB));
        const float swing = fabsf(v - _zeroVoltage[keyB]);
        if (swing > getEffectiveThresholdDelta(keyB) && counts[keyB] != 0xFFFF) counts[keyB]++;
      }
    }
  }

  // Summarize top offenders
  int total = 0;
  int topKey[10];
  int topCnt[10];
  for (int i = 0; i < 10; ++i) { topKey[i] = -1; topCnt[i] = -1; }

  for (int k = _fromKey; k <= _toKey; ++k) {
    const int c = counts[k];
    total += c;
    for (int i = 0; i < 10; ++i) {
      if (c > topCnt[i]) {
        for (int j = 9; j > i; --j) { topCnt[j] = topCnt[j-1]; topKey[j] = topKey[j-1]; }
        topCnt[i] = c;
        topKey[i] = k;
        break;
      }
    }
  }

  if (_outputFormat) {
    beginJson("idle");
    printJsonKV("seconds", seconds);
    printJsonKV("from", _fromKey);
    printJsonKV("to", _toKey);
    printJsonKV("total", total, true);
    endJson();
    for (int i = 0; i < 10; ++i) {
      if (topKey[i] < 0) break;
      beginJson("idle_top");
      printJsonKV("rank", i + 1);
      printJsonKV("key", topKey[i]);
      printJsonKV("count", topCnt[i], true);
      endJson();
    }
  } else {
    _println("idle seconds=%d total=%d", seconds, total);
    for (int i = 0; i < 10; ++i) {
      if (topKey[i] < 0) break;
      _println("idle top %d: key=%d count=%d", i + 1, topKey[i] + 1, topCnt[i]);
    }
  }

  diagEnd();
}

void handleAutoTuneIdle(float* params, int count)
{
  mcpAck();
  diagBegin();

  // Deterministic staged autotune within ~5 minutes.
  // Params: autotune<shortS>,<midS>,<strictS>,<topK>
  const int shortS = (count >= 1) ? max(2, (int)params[0]) : 5;
  const int midS = (count >= 2) ? max(10, (int)params[1]) : 20;
  const int strictS = (count >= 3) ? max(30, (int)params[2]) : 60;
  const int topK = (count >= 4) ? max(1, min(4, (int)params[3])) : 2;

  const int origAvg = _measureAvgStandard;
  const int origSd  = _us_delay_after_mux;

  const float minThr = 0.03f;
  const float maxThr = 0.25f;
  const float marginShort = 0.005f;
  const float marginMid = 0.010f;
  const float marginStrict = 0.015f;

  // Search space (include larger averaging values for stability-first tuning).
  // Note: Arduino/Teensy analogReadAveraging officially supports up to 32.
  const int avgCandidates[] = { 4, 8, 16, 32, 64, 128 };
  const int sdCandidates[]  = { 5, 10, 20, 50, 100, 200 };

  struct GlobalCand {
    int avg;
    int sd;
    int total;
    float worstExcess;
  };
  GlobalCand best[4];
  int bestN = 0;

  static uint16_t counts[NUM_KEYS];
  static float maxExc[NUM_KEYS];
  static float thrOrig[NUM_KEYS];
  for (int k = 0; k < NUM_KEYS; ++k) thrOrig[k] = _threshold_delta[k];

  _autoHasRun = true;
  _autoRunComplete = false;
  _autoRunId++;
  _autoRunStartMs = millis();
  _autoFromKey = _fromKey;
  _autoToKey = _toKey;
  _autoOrigAvg = origAvg;
  _autoOrigSd = origSd;
  _autoTuneSeconds = shortS;
  _autoValidateSeconds = strictS;
  _autoTrialsRun = 0;
  _autoTrialsN = 0;
  _autoBestAvg = origAvg;
  _autoBestSd = origSd;
  _autoBestTuneTotal = 0x7FFFFFFF;
  _autoBestValidateTotal = 0x7FFFFFFF;
  _autoMaxThr = 0.0f;
  _autoMaxThrKey = -1;

  // --- Stage A: coarse global search using short window ---
  for (int ia = 0; ia < (int)(sizeof(avgCandidates)/sizeof(avgCandidates[0])); ++ia) {
    for (int isd = 0; isd < (int)(sizeof(sdCandidates)/sizeof(sdCandidates[0])); ++isd) {
      const int avg = avgCandidates[ia];
      const int sd = sdCandidates[isd];

      setAveragingRamOnly(avg);
      setMuxDelayRamOnly(sd);

      for (int k = _fromKey; k <= _toKey; ++k) {
        if (_hasZero[k]) _threshold_delta[k] = max(minThr, min(maxThr, thrOrig[k]));
      }

      const int total = idleAuditCountsAndMax(shortS, counts, maxExc);
      float worstExcess = 0.0f;
      for (int k = _fromKey; k <= _toKey; ++k) {
        if (!_hasZero[k]) continue;
        const float excess = maxExc[k] - _threshold_delta[k];
        if (excess > worstExcess) worstExcess = excess;
      }

      if (_autoTrialsN < AUTO_MAX_TRIALS) {
        _autoTrials[_autoTrialsN++] = AutoTrial{avg, sd, shortS, total, 0, 0.0f};
      }
      _autoTrialsRun++;

      // Insert into best[] if it improves (lower worstExcess, then total).
      if (bestN < topK) {
        best[bestN++] = GlobalCand{avg, sd, total, worstExcess};
      } else {
        int worstI = 0;
        for (int i = 1; i < bestN; ++i) {
          if (best[i].worstExcess > best[worstI].worstExcess) worstI = i;
          else if (best[i].worstExcess == best[worstI].worstExcess && best[i].total > best[worstI].total) worstI = i;
        }
        if (worstExcess < best[worstI].worstExcess || (worstExcess == best[worstI].worstExcess && total < best[worstI].total)) {
          best[worstI] = GlobalCand{avg, sd, total, worstExcess};
        }
      }
    }
  }

  // Sort best[] by worstExcess asc, then total asc, then prefer faster (lower avg, lower sd).
  for (int i = 0; i < bestN; ++i) {
    for (int j = i + 1; j < bestN; ++j) {
      bool swap = false;
      if (best[j].worstExcess < best[i].worstExcess) swap = true;
      else if (best[j].worstExcess == best[i].worstExcess) {
        if (best[j].total < best[i].total) swap = true;
        else if (best[j].total == best[i].total) {
          if (best[j].avg < best[i].avg) swap = true;
          else if (best[j].avg == best[i].avg && best[j].sd < best[i].sd) swap = true;
        }
      }
      if (swap) { GlobalCand tmp = best[i]; best[i] = best[j]; best[j] = tmp; }
    }
  }

  // Always pick a baseline best candidate from Stage A so autodump never shows INT_MAX.
  if (bestN > 0) {
    _autoBestAvg = best[0].avg;
    _autoBestSd = best[0].sd;
    _autoBestTuneTotal = best[0].total;
    _autoBestValidateTotal = best[0].total; // placeholder until strict runs
  }

  // --- Stage B/C: refine candidates with mid window and strict validation ---
  int finalValidateTotal = (bestN > 0) ? best[0].total : 0x7FFFFFFF;
  float maxThrSeen = 0.0f;
  int maxThrKey = -1;

  for (int bi = 0; bi < bestN; ++bi) {
    const int avg = best[bi].avg;
    const int sd = best[bi].sd;
    setAveragingRamOnly(avg);
    setMuxDelayRamOnly(sd);

    // Reset to calibrated thresholds
    for (int k = _fromKey; k <= _toKey; ++k) {
      if (_hasZero[k]) _threshold_delta[k] = max(minThr, min(maxThr, thrOrig[k]));
    }

    // Mid pass: measure and set thresholds one-shot
    (void)idleAuditCountsAndMax(midS, counts, maxExc);
    int adjustedKeys = 0;
    maxThrSeen = 0.0f;
    maxThrKey = -1;
    for (int k = _fromKey; k <= _toKey; ++k) {
      _autoThrBefore[k] = _threshold_delta[k];
      _autoMaxExcursion[k] = maxExc[k];
      if (!_hasZero[k]) { _autoThrAfter[k] = _threshold_delta[k]; continue; }
      float thrNew = _threshold_delta[k];
      if (maxExc[k] > thrNew) thrNew = min(maxThr, max(maxExc[k] + marginMid, thrNew));
      if (thrNew > _threshold_delta[k] + 1e-6f) adjustedKeys++;
      _threshold_delta[k] = thrNew;
      _autoThrAfter[k] = thrNew;
      if (thrNew > maxThrSeen) { maxThrSeen = thrNew; maxThrKey = k; }
    }

    // Verify mid stability
    const int midTotalAfter = idleAuditCounts(midS, counts);
    if (_autoTrialsN < AUTO_MAX_TRIALS) _autoTrials[_autoTrialsN++] = AutoTrial{avg, sd, midS, midTotalAfter, adjustedKeys, maxThrSeen};
    _autoTrialsRun++;
    // Do not require mid pass to be perfect. Strict phase will drive thresholds up.

    // Strict validation: iterate. If offenders look like carryover glitches,
    // try bumping sd first (RAM-only) and re-run strict.
    int strictTotal = idleAuditCountsAndMax(strictS, counts, maxExc);
    for (int iter = 0; iter < 4; ++iter) {
      // Store counts from the current strict pass so spike diagnostics can run.
      memcpy(_autoCountsBest, counts, sizeof(counts));
      if (strictTotal == 0) break;

      // Spike diagnostics for current offenders (short)
      spikeDiagStrictOffenders(8);

      int carryN = 0;
      for (int i = 0; i < _autoSpikeN; ++i) if (_autoSpikeCarryoverLikely[i]) carryN++;

      bool bumpedSd = false;
      if (carryN >= 2) {
        // Increase sd one notch and retry strict without changing thresholds.
        const int sdStep[] = {5, 10, 20, 50, 100, 200};
        int cur = _us_delay_after_mux;
        int next = cur;
        for (int si = 0; si < (int)(sizeof(sdStep)/sizeof(sdStep[0])); ++si) {
          if (sdStep[si] == cur && si + 1 < (int)(sizeof(sdStep)/sizeof(sdStep[0]))) { next = sdStep[si+1]; break; }
        }
        if (next != cur) {
          setMuxDelayRamOnly(next);
          bumpedSd = true;
        }
      }

      int adj = 0;
      if (!bumpedSd) {
        for (int k = _fromKey; k <= _toKey; ++k) {
          if (!_hasZero[k]) continue;
          if (counts[k] == 0) continue;
          float thrNew = _threshold_delta[k];
          if (maxExc[k] > thrNew) thrNew = min(maxThr, max(maxExc[k] + marginStrict, thrNew));
          if (thrNew > _threshold_delta[k] + 1e-6f) adj++;
          _threshold_delta[k] = thrNew;
          if (thrNew > maxThrSeen) { maxThrSeen = thrNew; maxThrKey = k; }
        }
      }

      if (_autoTrialsN < AUTO_MAX_TRIALS) _autoTrials[_autoTrialsN++] = AutoTrial{avg, _us_delay_after_mux, strictS, strictTotal, adj, maxThrSeen};
      _autoTrialsRun++;

      // Re-run strict measurement after adjustment or sd bump
      strictTotal = idleAuditCountsAndMax(strictS, counts, maxExc);
    }

    // If this candidate improved, persist its state into the autotune run log.
    if (strictTotal < finalValidateTotal) {
      finalValidateTotal = strictTotal;
      _autoBestAvg = avg;
      // Use current sd (may have been bumped during strict loop)
      _autoBestSd = _us_delay_after_mux;
      _autoBestTuneTotal = best[bi].total;
      _autoBestValidateTotal = strictTotal;
      _autoMaxThr = maxThrSeen;
      _autoMaxThrKey = maxThrKey;
      memcpy(_autoCountsBest, counts, sizeof(counts));

      // Capture the per-key arrays and spike diagnostics for this best candidate.
      // Note: _autoThrBefore/_autoThrAfter/_autoMaxExcursion already reflect the most recent mid pass.
      // Refresh strict max excursion into _autoMaxExcursion for offenders.
      for (int k = _fromKey; k <= _toKey; ++k) {
        _autoMaxExcursion[k] = maxExc[k];
      }
      spikeDiagStrictOffenders(8);
    }

    if (finalValidateTotal == 0) break;
  }

  // Apply best candidate settings in RAM
  setAveragingRamOnly(_autoBestAvg);
  setMuxDelayRamOnly(_autoBestSd);

  // Record update rate + dynamics metrics into the run header fields.
  // (These will be emitted by autodump without spamming during the run.)
  {
    const uint32_t ms = 2000;
    const bool diagWas = _diagActive;
    _diagActive = false;
    _scanPasses = 0;
    _scanKeyReads = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
      scanKeysNormal();
    }
    _diagActive = diagWas;
    const float secs = (float)ms / 1000.0f;
    _autoKeysPerSec = secs > 0 ? ((float)_scanKeyReads / secs) : 0.0f;
    _autoRevisitMsEst = _autoKeysPerSec > 0 ? (1000.0f * (float)(_toKey - _fromKey + 1) / _autoKeysPerSec) : 0.0f;
  }

  {
    computeDynamicsMetrics(_autoOverMin, _autoOverMax, _autoOverMinKey, _autoOverMaxKey);
  }

  _autoRunComplete = true;

  diagEnd();
}

void handleRuntimePerf(float* params, int count)
{
  mcpAck();
  diagBegin();

  const int seconds = (count >= 1) ? max(1, (int)params[0]) : 5;
  const uint32_t ms = (uint32_t)seconds * 1000UL;

  _scanPasses = 0;
  _scanKeyReads = 0;
  const uint32_t t0 = millis();
  // Temporarily run the normal scan in a tight loop for measurement.
  // This approximates runtime load without MIDI/LED extras.
  while (millis() - t0 < ms) {
    scanKeysNormal();
  }

  const float secs = (float)ms / 1000.0f;
  const float passesPerSec = (float)_scanPasses / secs;
  const float readsPerSec = (float)_scanKeyReads / secs;
  const float revisitMs = readsPerSec > 0 ? (1000.0f * (float)(_toKey - _fromKey + 1) / readsPerSec) : 0.0f;

  if (_outputFormat) {
    beginJson("rperf");
    printJsonKV("seconds", seconds);
    printJsonKV("from", _fromKey);
    printJsonKV("to", _toKey);
    printJsonKV("passesPerSec", passesPerSec);
    printJsonKV("keysPerSec", readsPerSec);
    printJsonKV("revisitMsEst", revisitMs, true);
    endJson();
  } else {
    _println("rperf seconds=%d keys/s=%.1f passes/s=%.2f revisitMsEst=%.2f", seconds, readsPerSec, passesPerSec, revisitMs);
  }

  diagEnd();
}

void handleTest0(float* params, int count)
{
  (void)params; (void)count;
  if (_outputFormat) {
    beginJson("t0");
    printJsonKV("avg", _measureAvgStandard);
    printJsonKV("sd_us", _us_delay_after_mux);
    printJsonKV("range_from", _fromKey);
    printJsonKV("range_to", _toKey, true);
    endJson();
  } else {
    _println("t0 avg=%d sd_us=%d range=%d..%d", _measureAvgStandard, _us_delay_after_mux, _fromKey + 1, _toKey + 1);
  }
}

struct TestbResult {
  int key;
  float baseline;
  float sigma;
  float thr;
  float maxSwing;
  int pol; // -1 or +1 (sign of first threshold crossing)
  int hits;
  int detected;
  int detectMs;
};

static bool waitForPress(int key, uint32_t timeoutMs, TestbResult &out)
{
  // Robust press detector that does NOT require prior calibration.
  // 1) measure baseline + sigma for a short window
  // 2) consider "pressed" only if abs(v-baseline) exceeds a noise-based threshold
  //    for a few consecutive samples.

  const float minThr = 0.03f; // 30 mV
  const float kSigma = 8.0f;
  const int consecutive = 3;

  // Assume mux is already set to this key when called.
  // Baseline window (LED blue = hands off)
  setColorAll(0, 0, 0);
  setColor(key, 0, 0, 255);
  updateAllLEDs();

  const uint32_t tBase0 = millis();
  const uint32_t baseMs = 400;
  uint32_t n = 0;
  float mean = 0.0f;
  float m2 = 0.0f;
  float vMin = 1e9f;
  float vMax = -1e9f;

  while (millis() - tBase0 < baseMs) {
    setKey(key);
    (void)getAdcUnscaledRawByIndex(ADC_UNSCALED1);
    (void)getAdcUnscaledRawByIndex(ADC_UNSCALED2);
    const float v = getAdcVoltage(getAdcUnscaled(key));
    if (v < vMin) vMin = v;
    if (v > vMax) vMax = v;
    ++n;
    const float delta = v - mean;
    mean += delta / n;
    m2 += delta * (v - mean);
    delay(2);
  }

  const float sigma = (n > 1) ? sqrtf(m2 / (n - 1)) : 0.0f;
  const float baseline = mean;
  const float thr = max(minThr, kSigma * sigma);

  // White = press now
  setColor(key, 255, 255, 255);
  updateAllLEDs();

  out.key = key;
  out.baseline = baseline;
  out.sigma = sigma;
  out.thr = thr;
  out.maxSwing = 0.0f;
  out.pol = 0;
  out.hits = 0;
  out.detected = 0;
  out.detectMs = -1;

  if (!_outputFormat) {
    _println("testb key=%d baseline=%.4fV sigma=%.6f thr=%.4fV (min=%.4f max=%.4f)",
             key + 1, baseline, sigma, thr, vMin, vMax);
  }

  // Detection window
  const uint32_t t0 = millis();
  int hit = 0;
  bool reportedCandidate = false;
  while (millis() - t0 < timeoutMs) {
    setKey(key);
    (void)getAdcUnscaledRawByIndex(ADC_UNSCALED1);
    (void)getAdcUnscaledRawByIndex(ADC_UNSCALED2);
    const float v = getAdcVoltage(getAdcUnscaled(key));
    const float swing = fabsf(v - baseline);
    if (swing > out.maxSwing) out.maxSwing = swing;
    if (swing > thr) {
      // Yellow = candidate press detected
      setColor(key, 255, 255, 0);
      updateAllLEDs();
      hit++;
      out.hits++;
      if (!reportedCandidate) {
        reportedCandidate = true;
        out.pol = (v >= baseline) ? 1 : -1;
        if (_outputFormat) {
          beginJson("testb_hit");
          printJsonKV("key", key);
          printJsonKV("v", v);
          printJsonKV("swing", swing);
          printJsonKV("thr", thr);
          printJsonKV("pol", out.pol);
          printJsonKV("ms", (int)(millis() - t0), true);
          endJson();
        }
      }
      if (hit >= consecutive) return true;
    } else {
      // Back to white while waiting
      setColor(key, 255, 255, 255);
      updateAllLEDs();
      hit = 0;
    }
    delay(2);
  }
  return false;
}

void handleTest1(float* params, int count)
{
  (void)params; (void)count;
  diagBegin();
  // Ensure full key range for this test so the board sentinel keys are not skipped
  // due to a previously selected interval.
  setKeyInterval(0, NUM_KEYS - 1);

  // Start cue: two short blue flashes
  for (int i = 0; i < 2; ++i) {
    setColorAll(0, 0, 80);
    updateAllLEDs();
    delay(150);
    setColorAll(0, 0, 0);
    updateAllLEDs();
    delay(150);
  }
  // Sentinel positions:
  // Subtest 1: board 1 keys 1..16 (on-board mux sanity)
  // Subtest 2: first key of boards 2..5 (board presence)
  // Subtest 3: sparse spread on board 1 (17,33,49) as additional mux coverage
  static int sentinels[16 + 4 + 3];
  int nSentinels = 0;
  // board 1: keys 1..16
  for (int k = 0; k < 16; ++k) sentinels[nSentinels++] = k;
  // board first keys (skip duplicate 0)
  sentinels[nSentinels++] = 56;
  sentinels[nSentinels++] = 112;
  sentinels[nSentinels++] = 168;
  sentinels[nSentinels++] = 224;
  // board 1: keys 17,33,49 (0-based 16,32,48)
  sentinels[nSentinels++] = 16;
  sentinels[nSentinels++] = 32;
  sentinels[nSentinels++] = 48;

  if (_outputFormat) {
    beginJson("testb");
    printJsonKV("sentinels", nSentinels, true);
    endJson();
  } else {
    _println("Test B (sentinel press). Instructions:");
    _println("- For each lit key: keep hands off for ~0.5s, then press once firmly.");
    _println("- White = press key now; Green = pass; Red = retry/fail.");
    _println("- Timeout per key: 8 seconds.");
  }

  int passCount = 0;
  int failCount = 0;
  bool okAll = true;
  for (int i = 0; i < nSentinels; ++i) {
    const int key = sentinels[i];
    if (key < 0 || key >= NUM_KEYS) continue;
    // waitForPress() drives LEDs for baseline/press

    int board, boardKey;
    getBoardAndBoardKey(key, board, boardKey);

    if (!_outputFormat) {
      _println("Press sentinel key %d (board %d key %d)", key + 1, board + 1, boardKey + 1);
    }

    TestbResult res;
    const bool ok = waitForPress(key, 8000, res);

    if (_outputFormat) {
      beginJson("testb_row");
      printJsonKV("key", key);
      printJsonKV("board", board);
      printJsonKV("boardKey", boardKey);
      printJsonKV("ok", ok ? 1 : 0);
      printJsonKV("baseline", res.baseline);
      printJsonKV("sigma", res.sigma);
      printJsonKV("thr", res.thr);
      printJsonKV("maxSwing", res.maxSwing);
      printJsonKV("pol", res.pol);
      printJsonKV("hits", res.hits, true);
      endJson();
    } else {
      _println("testb row key=%d board=%d boardKey=%d ok=%d baseline=%.4f sigma=%.6f thr=%.4f maxSwing=%.4f hits=%d",
               key + 1, board + 1, boardKey + 1, ok ? 1 : 0,
               res.baseline, res.sigma, res.thr, res.maxSwing, res.hits);
    }

    if (!ok) { okAll = false; failCount++; } else { passCount++; }
    setColor(key, ok ? 0 : 255, ok ? 255 : 0, 0);
    updateAllLEDs();
    delay(ok ? 250 : 500);
  }

  if (_outputFormat) {
    beginJson("testb_done");
    printJsonKV("passCount", passCount);
    printJsonKV("failCount", failCount);
    printJsonKV("total", passCount + failCount);
    printJsonKV("pass", okAll ? 1 : 0, true);
    endJson();
  } else {
    _println("testb %s", okAll ? "PASS" : "FAIL");
    _println("testb summary pass=%d fail=%d total=%d", passCount, failCount, passCount + failCount);
  }

  // Finish animation: all keys blue, then restore
  setColorAll(0, 0, 0);
  updateAllLEDs();
  for (int i = 0; i < 3; ++i) {
    setColorAll(0, 0, 80);
    updateAllLEDs();
    delay(200);
    setColorAll(0, 0, 0);
    updateAllLEDs();
    delay(200);
  }
  setColorAll(0, 0, 80);
  updateAllLEDs();
  delay(800);

  diagEnd();
}

void handleTest2(float* params, int count)
{
  // settle sweep uses sts; pick two sentinels by default
  int keyA = 0;
  int keyB = 56;
  if (count >= 2) {
    keyA = (int)params[0] - 1;
    keyB = (int)params[1] - 1;
  }
  float p[2] = {(float)(keyA + 1), (float)(keyB + 1)};
  handleSettleSweep(p, 2);
}

void handleTest3(float* params, int count)
{
  const int seconds = (count >= 1) ? max(1, (int)params[0]) : 2;
  float p[1] = {(float)seconds};
  handleProblemReport(p, 1);
}

void handleTest6(float* params, int count)
{
  mcpAck();
  diagBegin();
  (void)params; (void)count;
  const int avgs[] = {1, 2, 4, 8, 16, 32};
  const int ms = 1000;

  if (_outputFormat) {
    beginJson("t6");
    printJsonKV("ms", ms);
    printJsonKV("steps", (int)(sizeof(avgs)/sizeof(avgs[0])), true);
    endJson();
  } else {
    _println("t6 perf sweep");
    Serial.println("avg,keysPerSec,sigmaP95");
  }

  // Use noise sigma snapshot for p95 on the current range
  for (int i = 0; i < (int)(sizeof(avgs)/sizeof(avgs[0])); ++i) {
    _measureAvgStandard = avgs[i];
    analogReadAveraging(_measureAvgStandard);

    // rate
    // capture rate into local by running an internal copy
    const int OFFSET = BOARDS_PER_ADC_CHANNEL * NUM_KEYS_PER_BOARD;
    uint32_t steps = 0;
    uint32_t reads = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < (uint32_t)ms) {
      for (int muxPos = 0; muxPos < OFFSET; ++muxPos) {
        setKey(muxPos);
        const int keyA = muxPos;
        const int keyB = muxPos + OFFSET;
        if (keyA >= _fromKey && keyA <= _toKey) { (void)getAdcUnscaled(keyA); ++reads; }
        if (keyB >= _fromKey && keyB <= _toKey) { (void)getAdcUnscaled(keyB); ++reads; }
        ++steps;
      }
    }
    const float secs = (float)ms / 1000.0f;
    const float keysPerSec = (reads / secs);

    // noise quick (200ms)
    scanNoise(200);
    // compute p95 sigma
    static float tmp[NUM_KEYS];
    int n = 0;
    for (int k = _fromKey; k <= _toKey; ++k) tmp[n++] = _noiseUnscaled[k];
    // partial sort for p95
    const int idx = (int)(0.95f * (n - 1));
    for (int a = 0; a <= idx; ++a) {
      int best = a;
      for (int b = a + 1; b < n; ++b) if (tmp[b] < tmp[best]) best = b;
      float t = tmp[a]; tmp[a] = tmp[best]; tmp[best] = t;
    }
    const float sigmaP95 = tmp[idx];

    if (_outputFormat) {
      beginJson("t6_row");
      printJsonKV("avg", _measureAvgStandard);
      printJsonKV("keysPerSec", keysPerSec);
      printJsonKV("sigmaP95", sigmaP95, true);
      endJson();
    } else {
      Serial.print(_measureAvgStandard);
      Serial.print(",");
      Serial.print(keysPerSec, 1);
      Serial.print(",");
      Serial.println(sigmaP95, 6);
    }
  }

  diagEnd();
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

