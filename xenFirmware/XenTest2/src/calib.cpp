#include "xen.h"
#include "calib.h"
#include "jsonl.h"
#include "midi.h"
#include "utils.h"
#include "mux.h"
#include "adc.h"
#include "sdcard.h"
#include "led.h"
#include "main.h"

float _threshold_delta[NUM_KEYS];
// Autotune threshold overlay (RAM): >0 overrides calibration threshold.
float _autotune_threshold_delta[NUM_KEYS];
float _zeroVoltage[NUM_KEYS];
bool _hasZero[NUM_KEYS];

// Baseline drift tracker state (idle-only updates while not playing).
static bool _baselineKeyActive[NUM_KEYS];
static elapsedMillis _baselineCooldownMs;
static const uint32_t _baselineCooldownHoldMs = 1000;

static inline bool baselineCooldownActive() { return _baselineCooldownMs < _baselineCooldownHoldMs; }
static inline bool isKeyActiveForBaseline(int key) { return _baselineKeyActive[key]; }

// --- Boot baseline drift capture ---
static float _bootZeroVoltage[NUM_KEYS];
static bool _bootZeroValid[NUM_KEYS];

static float _bootBoardDelta[NUM_BOARDS];
static bool _bootBoardDeltaValid[NUM_BOARDS];

static bool _bootDriftHasCache = false;
static int _bootDriftNCalib = 0;
static int _bootDriftNBootValid = 0;
static int _bootDriftNBootMissing = 0;
static float _bootDriftAvgAbs = 0.0f;
static float _bootDriftMedAbs = 0.0f;
static float _bootDriftMaxAbs = 0.0f;
static int _bootDriftMaxKey = -1;
static float _bootDriftMedDelta = 0.0f;

static void bootDriftCacheReset()
{
  _bootDriftHasCache = false;
  _bootDriftNCalib = 0;
  _bootDriftNBootValid = 0;
  _bootDriftNBootMissing = 0;
  _bootDriftAvgAbs = 0.0f;
  _bootDriftMedAbs = 0.0f;
  _bootDriftMaxAbs = 0.0f;
  _bootDriftMaxKey = -1;
  _bootDriftMedDelta = 0.0f;
  for (int b = 0; b < NUM_BOARDS; ++b) {
    _bootBoardDelta[b] = 0.0f;
    _bootBoardDeltaValid[b] = false;
  }
  memset(_bootZeroVoltage, 0, sizeof(_bootZeroVoltage));
  memset(_bootZeroValid, 0, sizeof(_bootZeroValid));
}

static void bootDriftPrintSummaryImpl()
{
  if (!_bootDriftHasCache) {
    if (_outputFormat) {
      Serial.println("{\"type\":\"bootdrift_sum\",\"ok\":0,\"error\":\"no_cache\"}");
    } else {
      _println("bootdrift ok=0 error=no_cache");
    }
    return;
  }
  if (_outputFormat) {
    Serial.print("{\"type\":\"bootdrift_sum\",\"ok\":1,\"nCalib\":");
    Serial.print(_bootDriftNCalib);
    Serial.print(",\"nBootValid\":");
    Serial.print(_bootDriftNBootValid);
    Serial.print(",\"nBootMissing\":");
    Serial.print(_bootDriftNBootMissing);
    Serial.print(",\"avgAbs\":");
    Serial.print(_bootDriftAvgAbs, 6);
    Serial.print(",\"medAbs\":");
    Serial.print(_bootDriftMedAbs, 6);
    Serial.print(",\"maxAbs\":");
    Serial.print(_bootDriftMaxAbs, 6);
    Serial.print(",\"maxKey\":");
    Serial.print(_bootDriftMaxKey);
    Serial.print(",\"medDelta\":");
    Serial.print(_bootDriftMedDelta, 6);
    Serial.println("}");
  } else {
    _println("bootdrift nCalib=%d valid=%d missing=%d avgAbs=%.6fV medAbs=%.6fV maxAbs=%.6fV key=%d medDelta=%.6fV",
            _bootDriftNCalib,
            _bootDriftNBootValid,
            _bootDriftNBootMissing,
            _bootDriftAvgAbs,
            _bootDriftMedAbs,
            _bootDriftMaxAbs,
            _bootDriftMaxKey + 1,
            _bootDriftMedDelta);
  }
}

static int clampInt(int v, int lo, int hi) { if (v < lo) return lo; if (v > hi) return hi; return v; }

static float medianDeltaFromHist(const uint16_t* hist, int bins, int n, int offset)
{
  if (n <= 0) return 0.0f;
  int target = (n + 1) / 2;
  int acc = 0;
  for (int i = 0; i < bins; ++i) {
    acc += hist[i];
    if (acc >= target) {
      const int mv = i - offset;
      return (float)mv / 1000.0f;
    }
  }
  return 0.0f;
}

static void computeBoardDeltas(const float* calibZero)
{
  // Signed delta histogram in mV: [-999..999]
  static uint16_t histAll[1999];
  memset(histAll, 0, sizeof(histAll));
  int nAll = 0;

  static uint16_t histBoard[NUM_BOARDS][1999];
  static int nBoard[NUM_BOARDS];
  for (int b = 0; b < NUM_BOARDS; ++b) {
    memset(histBoard[b], 0, sizeof(histBoard[b]));
    nBoard[b] = 0;
  }

  for (int k = 0; k < NUM_KEYS; ++k) {
    if (!_hasZero[k]) continue;
    if (!_bootZeroValid[k]) continue;
    const float d = _bootZeroVoltage[k] - calibZero[k];
    int mv = (int)(d * 1000.0f + (d >= 0 ? 0.5f : -0.5f));
    mv = clampInt(mv, -999, 999);
    histAll[mv + 999]++;
    nAll++;

    int board, boardKey;
    getBoardAndBoardKey(k, board, boardKey);
    if (board >= 0 && board < NUM_BOARDS) {
      histBoard[board][mv + 999]++;
      nBoard[board]++;
    }
  }

  const float medAll = medianDeltaFromHist(histAll, 1999, nAll, 999);
  _bootDriftMedDelta = medAll;

  for (int b = 0; b < NUM_BOARDS; ++b) {
    if (nBoard[b] >= 8) {
      _bootBoardDelta[b] = medianDeltaFromHist(histBoard[b], 1999, nBoard[b], 999);
      _bootBoardDeltaValid[b] = true;
    } else {
      _bootBoardDelta[b] = medAll;
      _bootBoardDeltaValid[b] = (nAll > 0);
    }
  }
}

static void applyBootDriftToBaseline(const float* calibZero)
{
  // Recommended behavior:
  // - If per-key boot baseline exists: use it.
  // - Else: use per-board median delta fallback.
  for (int k = 0; k < NUM_KEYS; ++k) {
    if (!_hasZero[k]) continue;
    if (_bootZeroValid[k]) {
      _zeroVoltage[k] = _bootZeroVoltage[k];
      continue;
    }
    int board, boardKey;
    getBoardAndBoardKey(k, board, boardKey);
    float d = _bootDriftMedDelta;
    if (board >= 0 && board < NUM_BOARDS && _bootBoardDeltaValid[board]) d = _bootBoardDelta[board];
    _zeroVoltage[k] = calibZero[k] + d;
  }
}

static void captureBootBaselinesInternal(bool verbose)
{
  // Requires calibration baseline present (calibZero snapshot).
  static float calibZero[NUM_KEYS];
  for (int k = 0; k < NUM_KEYS; ++k) calibZero[k] = _zeroVoltage[k];

  // Measure boot baselines
  memset(_bootZeroVoltage, 0, sizeof(_bootZeroVoltage));
  memset(_bootZeroValid, 0, sizeof(_bootZeroValid));

  analogReadAveraging(_measureAvgStandard > 32 ? 32 : _measureAvgStandard);
  const int OFFSET = BOARDS_PER_ADC_CHANNEL * NUM_KEYS_PER_BOARD;
  const int samplesPerKey = 12;
  const int settleUs = max(20, _us_delay_after_mux);

  for (int muxPos = 0; muxPos < OFFSET; ++muxPos) {
    const int keyA = muxPos;
    const int keyB = muxPos + OFFSET;
    const bool doA = (keyA >= 0 && keyA < NUM_KEYS) && _hasZero[keyA];
    const bool doB = (keyB >= 0 && keyB < NUM_KEYS) && _hasZero[keyB];
    if (!doA && !doB) continue;

    setKey(muxPos);
    delayMicroseconds(settleUs);
    (void)getAdcUnscaledRawByIndex(ADC_UNSCALED1);
    (void)getAdcUnscaledRawByIndex(ADC_UNSCALED2);
    float sumA = 0.0f, sumB = 0.0f;
    for (int i = 0; i < samplesPerKey; ++i) {
      if (doA) sumA += getAdcVoltage(getAdcUnscaled(keyA));
      if (doB) sumB += getAdcVoltage(getAdcUnscaled(keyB));
    }
    if (doA) { _bootZeroVoltage[keyA] = sumA / (float)samplesPerKey; _bootZeroValid[keyA] = true; }
    if (doB) { _bootZeroVoltage[keyB] = sumB / (float)samplesPerKey; _bootZeroValid[keyB] = true; }
  }

  // Drift stats (abs) + median abs
  static uint16_t histAbsMv[1000];
  memset(histAbsMv, 0, sizeof(histAbsMv));
  int n = 0;
  float sumAbs = 0.0f;
  float maxAbs = 0.0f;
  int maxKey = -1;
  int nCalib = 0;
  for (int k = 0; k < NUM_KEYS; ++k) if (_hasZero[k]) nCalib++;

  for (int k = 0; k < NUM_KEYS; ++k) {
    if (!_hasZero[k]) continue;
    if (!_bootZeroValid[k]) continue;
    const float d = _bootZeroVoltage[k] - calibZero[k];
    const float ad = fabsf(d);
    n++;
    sumAbs += ad;
    if (ad > maxAbs) { maxAbs = ad; maxKey = k; }
    int mv = (int)(ad * 1000.0f + 0.5f);
    if (mv < 0) mv = 0;
    if (mv > 999) mv = 999;
    histAbsMv[mv]++;
  }
  int medMv = 0;
  if (n > 0) {
    const int target = (n + 1) / 2;
    int acc = 0;
    for (int mv = 0; mv < 1000; ++mv) { acc += histAbsMv[mv]; if (acc >= target) { medMv = mv; break; } }
  }

  _bootDriftHasCache = true;
  _bootDriftNCalib = nCalib;
  _bootDriftNBootValid = n;
  _bootDriftNBootMissing = max(0, nCalib - n);
  _bootDriftAvgAbs = (n > 0) ? (sumAbs / (float)n) : 0.0f;
  _bootDriftMedAbs = (float)medMv / 1000.0f;
  _bootDriftMaxAbs = maxAbs;
  _bootDriftMaxKey = maxKey;

  computeBoardDeltas(calibZero);
  applyBootDriftToBaseline(calibZero);

  // Minimal summary always; verbose only when requested.
  bootDriftPrintSummaryImpl();
  (void)verbose;
}

void bootDriftPrintSummary()
{
  bootDriftPrintSummaryImpl();
}

void driftReset()
{
  if (!_calibAutoLoadOk) {
    if (_outputFormat) {
      Serial.println("{\"type\":\"bootdrift_sum\",\"ok\":0,\"error\":\"calib_not_loaded\"}");
    } else {
      _println("bootdrift ok=0 error=calib_not_loaded");
    }
    return;
  }
  captureBootBaselinesInternal(true);
}

int _currentCalibrationKey = 0;
int _calibLoopState = STATE_CALIBRATION_START;
float _maxSwing[NUM_KEYS];
short _polarization[NUM_KEYS];
bool _manualCalibration = false;
float _noiseUnscaled[NUM_KEYS];

bool _calibAutoLoadOk = false;
bool _bootedInDebugMode = false;
bool _calibDirty = false;

// Calibration bookkeeping (skips/fails)
static bool _calibSkipped[NUM_KEYS];
static bool _calibFailed[NUM_KEYS];
static uint32_t _calibWaitOnStartMs = 0;
static bool _calibSelfTriggering[NUM_KEYS];
static bool _calibNotReleasing[NUM_KEYS];
static bool _calibSlightlyStuck[NUM_KEYS];
static uint32_t _calibReleaseMs[NUM_KEYS];

// Tracks if the current issue arrays contain a completed run summary.
static bool _calibRunHasSummary = false;

// Per-key state timers (avoid static carry-over between keys)
static uint32_t _calibHoldStartMs = 0;
static uint32_t _calibOnStartMs = 0;

// Enforce initial/between-key settling before baseline capture
static bool _calibNeedSettle = false;
static uint32_t _calibSettleStartMs = 0;

volatile uint32_t _scanPasses = 0;
volatile uint32_t _scanKeyReads = 0;

int _measureAvgStandard = 32;
int _measureAvgCalibration = 32;

void setupCalibration() {
   memset(_autotune_threshold_delta, 0, sizeof(_autotune_threshold_delta));
   bootDriftCacheReset();
   bool ok = !_manualCalibration && loadCalibrationCSV();
   _calibAutoLoadOk = ok;
   _manualCalibration = false;

   // Compute/apply boot drift compensation once calibration is loaded.
   if (ok) {
     captureBootBaselinesInternal(false);
   }
   
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

static inline void scanKey(int key)
{
    // Support software averaging for values above what the ADC hardware supports.
    // analogReadAveraging() caps at 32 on Teensy 4.x, so avg=64/128 mean:
    // take multiple reads and average in software.
    const int avg = _measureAvgStandard;
    float v = 0.0f;
    if (avg > 32) {
        const int reps = (avg + 31) / 32; // 64->2, 128->4
        float sum = 0.0f;
        for (int i = 0; i < reps; ++i) {
            const int adc = getAdcUnscaled(key);
            sum += getAdcVoltage(adc);
        }
        v = sum / (float)reps;
    } else {
        const int adc = getAdcUnscaled(key);
        v = getAdcVoltage(adc);
    }
    // Baseline drift tracking (idle-only).
    // Update only when the key is idle and the system hasn't played recently.
    
    if (_hasZero[key] && !baselineCooldownActive() && !isKeyActiveForBaseline(key)) {
        const float thr = getEffectiveThresholdDelta(key);
        const float gate = 0.5f * thr;
        const float swing = fabsf(v - _zeroVoltage[key]);
        if (swing < gate) {
            // Slow IIR towards the current reading (tuned for multi-second drift).
            const float alpha = 0.0025f;
            _zeroVoltage[key] = _zeroVoltage[key] + alpha * (v - _zeroVoltage[key]);
        }
    }

    peakDetect(v, key);
}

float getEffectiveThresholdDelta(int key)
{
    const float a = _autotune_threshold_delta[key];
    if (a > 1e-6f) return a;
    return _threshold_delta[key];
}

void scanKeysNormal() {
  if (_diagActive) return;
  while (usbMIDI.read()) {};

  const int OFFSET = BOARDS_PER_ADC_CHANNEL * NUM_KEYS_PER_BOARD; // 3 * 56 in our case

  for (int muxPos = 0; muxPos < OFFSET; ++muxPos) // each possible MUX address exactly once
  {
      const int keyA = muxPos;            // “lower” key that uses this MUX line
      const int keyB = muxPos + OFFSET;   // “upper” key that re-uses the same MUX line

      const bool doA = (keyA >= _fromKey) && (keyA <= _toKey);
      const bool doB = (keyB >= _fromKey) && (keyB <= _toKey);

      if (!doA && !doB)                   // nothing to do for this MUX address
          continue;
    
      setKey(muxPos);                     // one change of the analogue multiplexer
      delayMicroseconds(_us_delay_after_mux);

      // Discard-first-sample strategy to reduce carryover/ADC sample cap artifacts.
      (void)getAdcUnscaledRawByIndex(ADC_UNSCALED1);
      (void)getAdcUnscaledRawByIndex(ADC_UNSCALED2);

      if (doA) { scanKey(keyA); ++_scanKeyReads; }
      if (doB) { scanKey(keyB); ++_scanKeyReads; }
  }

  ++_scanPasses;
}

void scanNoise(int ms) {    
    // --- per-key running statistics (Welford) -------------------------------
    static float meanUnscaled[NUM_KEYS];
    static float m2Unscaled  [NUM_KEYS];
    static uint32_t n[NUM_KEYS];

    memset(meanUnscaled, 0, sizeof(meanUnscaled));
    memset(m2Unscaled,   0, sizeof(m2Unscaled));
    memset(n,            0, sizeof(n));

    const int OFFSET = BOARDS_PER_ADC_CHANNEL * NUM_KEYS_PER_BOARD;
    const uint32_t t0 = millis();

    //-----------------------------------------------------------------------
    while (millis() - t0 < (uint32_t)ms)        // run for the requested time
    {
        for (int muxPos = 0; muxPos < OFFSET; ++muxPos)
        {
            const int keyA = muxPos;
            const int keyB = muxPos + OFFSET;

            const bool doA = (keyA >= _fromKey) && (keyA <= _toKey);
            const bool doB = (keyB >= _fromKey) && (keyB <= _toKey);
            if (!doA && !doB) continue;

            setKey(muxPos);
            delayMicroseconds(_us_delay_after_mux);

            (void)getAdcUnscaledRawByIndex(ADC_UNSCALED1);
            (void)getAdcUnscaledRawByIndex(ADC_UNSCALED2);

            // ------------------------------- sample A ----------------------
            if (doA)
            {
                const float vRaw  = getAdcVoltage(getAdcUnscaled(keyA));

                ++n[keyA];

                float delta = vRaw - meanUnscaled[keyA];
                meanUnscaled[keyA] += delta / n[keyA];
                m2Unscaled[keyA]   += delta * (vRaw - meanUnscaled[keyA]);
            }

            // ------------------------------- sample B ----------------------
            if (doB)
            {
                const float vRaw  = getAdcVoltage(getAdcUnscaled(keyB));

                ++n[keyB];

                float delta = vRaw - meanUnscaled[keyB];
                meanUnscaled[keyB] += delta / n[keyB];
                m2Unscaled[keyB]   += delta * (vRaw - meanUnscaled[keyB]);
            }
        } // for muxPos
    }     // while time

    // ----------- convert M2 to σ and store in global output arrays ----------
    for (int k = 0; k < NUM_KEYS; ++k)
    {
        if (n[k] > 1) {
            _noiseUnscaled[k] = sqrtf(m2Unscaled[k] / (n[k] - 1));  // σRAW
        } else {
            _noiseUnscaled[k] = 0.0f;
        }
    }
}

void printNoiseLevels()
{
    const int sets = (NUM_KEYS + NUM_KEYS_PER_BOARD - 1) / NUM_KEYS_PER_BOARD;   // round-up

    for (int s = 0; s < sets; ++s)
    {
        const int first = s * NUM_KEYS_PER_BOARD;
        const int last  = min(first + NUM_KEYS_PER_BOARD, NUM_KEYS) - 1;

        float maxUnscaled = 0.0f;
        float sumUnscaled = 0.0f;
        int   cntUnscaled = 0;

        for (int k = first; k <= last; ++k)
        {
            const float nu = _noiseUnscaled[k];
            if (nu > maxUnscaled) maxUnscaled = nu;
            if (nu > 0.0f) { sumUnscaled += nu; ++cntUnscaled; }
        }

        const float avgUnscaled = cntUnscaled ? sumUnscaled / cntUnscaled : 0.0f;

        _println("Keys %3d-%3d  |  Unscaled σ ≤ %.6f  (σ̄ = %.6f)",
                 first + 1, last + 1,
                 maxUnscaled, avgUnscaled);
    }
}

//--------------------------------------------------------------------------
//  Map HSV → 8-bit RGB  (H in degrees, S & V 0-1).  Small and branch-free.
//
void hsvToRgb(float h, float s, float v,
                     uint8_t &r, uint8_t &g, uint8_t &b)
{
    h = fmodf(h, 360.0f);               // keep in 0‒360
    if (h < 0) h += 360.0f;

    const float c = v * s;
    const float h_ = h / 60.0f;
    const float x = c * (1.0f - fabsf(fmodf(h_, 2.0f) - 1.0f));
    float r1 = 0, g1 = 0, b1 = 0;

    if      (h_ < 1) { r1 = c; g1 = x; }
    else if (h_ < 2) { r1 = x; g1 = c; }
    else if (h_ < 3) { g1 = c; b1 = x; }
    else if (h_ < 4) { g1 = x; b1 = c; }
    else if (h_ < 5) { r1 = x; b1 = c; }
    else             { r1 = c; b1 = x; }

    const float m = v - c;
    r = uint8_t((r1 + m) * 255.0f + 0.5f);
    g = uint8_t((g1 + m) * 255.0f + 0.5f);
    b = uint8_t((b1 + m) * 255.0f + 0.5f);
}

//---------------------------------------------------------------------------
//  Paint the strip with a blue-to-red gradient representing noise level.
//---------------------------------------------------------------------------
void showNoise(int)
{
    const float *noise = _noiseUnscaled;

    // ----- find span (min / max) so we can normalise to 0‒1 -------------
    float nMin = noise[0], nMax = noise[0];
    for (int k = 1; k < NUM_KEYS; ++k) {
        if (noise[k] < nMin) nMin = noise[k];
        if (noise[k] > nMax) nMax = noise[k];
    }
    float span = nMax - nMin;
    if (span < 1e-9f) span = 1e-9f;     // prevent /0 when every key is identical

    // --------------------------------------------------------------------
    for (int k = 0; k < NUM_KEYS; ++k)
    {
        // 0 → blue (240°);  1 → red (0°)
        const float norm = (noise[k] - nMin) / span;      // 0‒1
        const float hue  = (1.0f - norm) * 240.0f;        // 240° → 0°

        uint8_t r, g, b;
        hsvToRgb(hue, 1.0f, 1.0f, r, g, b);

        LEDStrip->setPixelColor(k,
                                LEDStrip->gamma8(r),
                                LEDStrip->gamma8(g),
                                LEDStrip->gamma8(b));
    }

    LEDStrip->show();
}

void printCalibration(int key) {
    _println("[%d] MaxSwing=%f V, Pol=%d",
        key+1, 
        _maxSwing[key], 
        _polarization[key]);
    _println("[%d] Zero=%f", key+1, _zeroVoltage[key]);
    _println("[%d] Threshold=%f V", key+1, _threshold_delta[key]);
}

void setThreshold(int key, float threshold) {
    _threshold_delta[key]= threshold;
}

void clearCalibration(int fromKey, int toKey)
{
    if (fromKey < 0) fromKey = 0;
    if (toKey >= NUM_KEYS) toKey = NUM_KEYS - 1;
    if (toKey < fromKey) return;

    for (int k = fromKey; k <= toKey; ++k) {
        _hasZero[k] = false;
        _zeroVoltage[k] = 0.0f;
        _threshold_delta[k] = 0.0f;
        _maxSwing[k] = 0.0f;
        _polarization[k] = 0;
        _noiseUnscaled[k] = 0.0f;
    }

    _calibDirty = true;
}

void loopCalibrationStart() {
    bool ok = !_manualCalibration && loadCalibrationCSV();
    
    if (ok) {
      // Calibration already loaded; keep non-selected keys off and do not
      // paint a range highlight during calibration.
      setColorAll(0, 0, 0);
      updateAllLEDs();
      _calibLoopState = STATE_RUNNING;
      return;
    }

    // Clean slate only for selected range.
    clearCalibration(_fromKey, _toKey);
    for (int k = _fromKey; k <= _toKey; ++k) {
      _calibSkipped[k] = false;
      _calibFailed[k] = false;
      _calibSelfTriggering[k] = false;
      _calibNotReleasing[k] = false;
      _calibSlightlyStuck[k] = false;
      _calibReleaseMs[k] = 0;
    }

    _calibRunHasSummary = false;

    _calibHoldStartMs = 0;
    _calibOnStartMs = 0;

    _currentCalibrationKey = _fromKey;
    _calibNeedSettle = true;
    _calibSettleStartMs = millis();
    _calibLoopState = STATE_CALIBRATION_OFF;

    if (_outputFormat) {
      Serial.print("{\"type\":\"calib_start\",\"from\":");
      Serial.print(_fromKey);
      Serial.print(",\"to\":");
      Serial.print(_toKey);
      Serial.println("}");
    }
}
  
void loopCalibrationOff() {
    setKey(_currentCalibrationKey);
    analogReadAveraging(_measureAvgCalibration);

    // Match testb algorithm and logging as closely as possible.
    const float thresholdMin = 0.03f;
    const float kSigma = 8.0f;
    const uint32_t baseMs = 400;
    const uint32_t settleMs = 200;
    const int baselineRetries = 3;

    // Settling gate to ensure "clean slate" sampling after mux switch and key changes.
    if (_calibNeedSettle) {
      setColorAll(0, 0, 0);
      setColor(_currentCalibrationKey, 0, 0, 255);
      updateAllLEDs();

      if (millis() - _calibSettleStartMs < settleMs) {
        (void)getAdcUnscaledRawByIndex(ADC_UNSCALED1);
        (void)getAdcUnscaledRawByIndex(ADC_UNSCALED2);
        delay(5);
        return;
      }
      _calibNeedSettle = false;
    }

    // LED: blue = baseline (hands off)
    setColorAll(0, 0, 0);
    setColor(_currentCalibrationKey, 0, 0, 255);
    updateAllLEDs();

    float sigma = 0.0f;
    float baseline = 0.0f;
    float thr = thresholdMin;
    float vMin =  0.0f;
    float vMax =  0.0f;
    float baseSwingMax = 0.0f;
    bool okBaseline = false;

    for (int attempt = 0; attempt < baselineRetries; ++attempt) {
      // Discard-first-sample after mux switch to reduce carryover/settling artifacts.
      (void)getAdcUnscaledRawByIndex(ADC_UNSCALED1);
      (void)getAdcUnscaledRawByIndex(ADC_UNSCALED2);

      const uint32_t t0 = millis();
      uint32_t n = 0;
      float mean = 0.0f;
      float m2 = 0.0f;
      vMin =  1e9f;
      vMax = -1e9f;
      while (millis() - t0 < baseMs) {
          const float v = getAdcVoltage(getAdcUnscaled(_currentCalibrationKey));
          if (v < vMin) vMin = v;
          if (v > vMax) vMax = v;
          ++n;
          const float delta = v - mean;
          mean += delta / n;
          m2   += delta * (v - mean);
          delay(2);
      }

      sigma = (n > 1) ? sqrtf(m2 / (n - 1)) : 0.0f;
      baseline = mean;
      thr = max(thresholdMin, kSigma * sigma);
      baseSwingMax = max(fabsf(vMax - baseline), fabsf(vMin - baseline));

      if (baseSwingMax <= thr) {
        okBaseline = true;
        break;
      }

      // Retry after a short settle.
      delay(50);
    }

    if (!okBaseline) {
        _calibSelfTriggering[_currentCalibrationKey] = true;
        if (_outputFormat) {
          int board, boardKey;
          getBoardAndBoardKey(_currentCalibrationKey, board, boardKey);
          Serial.print("{\"type\":\"calib_row\",\"key\":");
          Serial.print(_currentCalibrationKey);
          Serial.print(",\"board\":");
          Serial.print(board);
          Serial.print(",\"boardKey\":");
          Serial.print(boardKey);
          Serial.print(",\"ok\":0,\"reason\":\"SelfTriggering\",");
          Serial.print("\"baseline\":");
          Serial.print(baseline, 6);
          Serial.print(",\"sigma\":");
          Serial.print(sigma, 6);
          Serial.print(",\"thr\":");
          Serial.print(thr, 6);
          Serial.print(",\"baseSwingMax\":");
          Serial.print(baseSwingMax, 6);
          Serial.println("}");
        } else {
          _println("calib SelfTriggering key=%d baseSwingMax=%.4f thr=%.4f", _currentCalibrationKey + 1, baseSwingMax, thr);
        }

        // LED: orange blink then advance
        setColor(_currentCalibrationKey, 255, 100, 0);
        updateAllLEDs();
        delay(200);
        setColor(_currentCalibrationKey, 0, 0, 0);
        updateAllLEDs();

        if (_currentCalibrationKey < _toKey) {
            _currentCalibrationKey++;
            _calibNeedSettle = true;
            _calibSettleStartMs = millis();
            _calibLoopState = STATE_CALIBRATION_OFF;
        } else {
            _calibLoopState = STATE_CALIBRATION_STOP;
        }
        return;
    }

    _threshold_delta[_currentCalibrationKey] = thr;
    _zeroVoltage[_currentCalibrationKey] = baseline;
    // Only mark the key as calibrated after a successful press+release.
    _hasZero[_currentCalibrationKey] = false;
    _maxSwing[_currentCalibrationKey] = 0.0f;
    _polarization[_currentCalibrationKey] = 0;
    _noiseUnscaled[_currentCalibrationKey] = sigma;

    _calibDirty = true;

    // Emit testb-style baseline log line too (calibration is a superset).
    if (_outputFormat) {
      Serial.print("{\"type\":\"testb\",\"key\":");
      Serial.print(_currentCalibrationKey);
      Serial.print(",\"baseline\":");
      Serial.print(baseline, 6);
      Serial.print(",\"sigma\":");
      Serial.print(sigma, 6);
      Serial.print(",\"thr\":");
      Serial.print(thr, 6);
      Serial.print(",\"min\":");
      Serial.print(vMin, 6);
      Serial.print(",\"max\":");
      Serial.print(vMax, 6);
      Serial.println("}");
    } else {
      _println("testb key=%d baseline=%.4fV sigma=%.6f thr=%.4fV (min=%.4f max=%.4f)",
               _currentCalibrationKey + 1, baseline, sigma, thr, vMin, vMax);
    }

    if (_outputFormat) {
      Serial.print("{\"type\":\"calib_base\",\"key\":");
      Serial.print(_currentCalibrationKey);
      Serial.print(",\"baseline\":");
      Serial.print(baseline, 6);
      Serial.print(",\"sigma\":");
      Serial.print(sigma, 6);
      Serial.print(",\"thr\":");
      Serial.print(thr, 6);
      Serial.print(",\"min\":");
      Serial.print(vMin, 6);
      Serial.print(",\"max\":");
      Serial.print(vMax, 6);
      Serial.println("}");
    } else {
      _println("calib_base key=%d baseline=%.4f sigma=%.6f thr=%.4f min=%.4f max=%.4f",
               _currentCalibrationKey + 1, baseline, sigma, thr, vMin, vMax);
    }

    // LED: white = press now
    setColor(_currentCalibrationKey, 255, 255, 255);
    updateAllLEDs();

    _calibWaitOnStartMs = millis();
    _calibHoldStartMs = 0;
    _calibOnStartMs = 0;

    _calibLoopState = STATE_CALIBRATION_WAIT_ON;
  }
  
void loopCalibrationWaitOn() {
    // Skip gesture: hold the current key (swing > threshold) continuously for >2 seconds.
    const uint32_t skipHoldMs = 2000;
    // (no static timers; per-session globals)

    int currentValue = getAdcUnscaled(_currentCalibrationKey);
    float currentVoltage = getAdcVoltage(currentValue);
    float swing = fabs(currentVoltage - _zeroVoltage[_currentCalibrationKey]);

    // If no press detected for a long time, mark as failed and advance.
    if (millis() - _calibWaitOnStartMs > 8000) {
        _calibFailed[_currentCalibrationKey] = true;
        if (_outputFormat) {
          int board, boardKey;
          getBoardAndBoardKey(_currentCalibrationKey, board, boardKey);
          Serial.print("{\"type\":\"calib_row\",\"key\":");
          Serial.print(_currentCalibrationKey);
          Serial.print(",\"board\":");
          Serial.print(board);
          Serial.print(",\"boardKey\":");
          Serial.print(boardKey);
           Serial.print(",\"ok\":0,\"reason\":\"NotTriggering\"}");
           Serial.println();
        }

        // LED: red flash then advance
        setColor(_currentCalibrationKey, 255, 0, 0);
        updateAllLEDs();
        delay(200);
        setColor(_currentCalibrationKey, 0, 0, 0);
        updateAllLEDs();

        if (_currentCalibrationKey < _toKey) {
            _currentCalibrationKey++;
            _calibLoopState = STATE_CALIBRATION_OFF;
        } else {
            _calibLoopState = STATE_CALIBRATION_STOP;
        }
        return;
    }
  
    if (swing > _threshold_delta[_currentCalibrationKey]) {
        // If user keeps holding beyond skipHoldMs, treat it as "skip".
        if (_calibHoldStartMs == 0) _calibHoldStartMs = millis();
        if (millis() - _calibHoldStartMs > skipHoldMs) {
            _calibSkipped[_currentCalibrationKey] = true;
            _calibHoldStartMs = 0;

            if (_outputFormat) {
              int board, boardKey;
              getBoardAndBoardKey(_currentCalibrationKey, board, boardKey);
              Serial.print("{\"type\":\"calib_row\",\"key\":");
              Serial.print(_currentCalibrationKey);
              Serial.print(",\"board\":");
              Serial.print(board);
              Serial.print(",\"boardKey\":");
              Serial.print(boardKey);
              Serial.print(",\"ok\":0,\"reason\":\"skipped\"}");
              Serial.println();
            }

            // LED: purple blink for skip
            setColor(_currentCalibrationKey, 180, 0, 180);
            updateAllLEDs();
            delay(200);
            setColor(_currentCalibrationKey, 0, 0, 0);
            updateAllLEDs();

            if (_currentCalibrationKey < _toKey) {
                _currentCalibrationKey++;
                _calibNeedSettle = true;
                _calibSettleStartMs = millis();
                _calibLoopState = STATE_CALIBRATION_OFF;
            } else {
                _calibLoopState = STATE_CALIBRATION_STOP;
            }
            return;
        }

        // LED: yellow = candidate press
        setColor(_currentCalibrationKey, 255, 255, 0);
        updateAllLEDs();
        _maxSwing[_currentCalibrationKey] = swing;

        if (_outputFormat) {
          Serial.print("{\"type\":\"calib_hit\",\"key\":");
          Serial.print(_currentCalibrationKey);
          Serial.print(",\"v\":");
          Serial.print(currentVoltage, 6);
          Serial.print(",\"swing\":");
          Serial.print(swing, 6);
          Serial.print(",\"thr\":");
          Serial.print(_threshold_delta[_currentCalibrationKey], 6);
          Serial.println("}");
        } else {
          _println("calib_hit key=%d v=%.4f swing=%.4f thr=%.4f", _currentCalibrationKey+1, currentVoltage, swing, _threshold_delta[_currentCalibrationKey]);
        }
        _calibLoopState = STATE_CALIBRATION_ON;
    } else {
        // reset hold timer if below threshold
        _calibHoldStartMs = 0;
    }
  }
  
void loopCalibrationOn() {
    int currentValue = getAdcUnscaled(_currentCalibrationKey);
    float currentVoltage = getAdcVoltage(currentValue);
    float swing = fabs(currentVoltage - _zeroVoltage[_currentCalibrationKey]);
  
    if (swing > _maxSwing[_currentCalibrationKey])
       _maxSwing[_currentCalibrationKey] = swing;
  
    _polarization[_currentCalibrationKey] = currentVoltage > _zeroVoltage[_currentCalibrationKey] ? 1 : -1;
  
    // Not releasing detection: if user never returns under threshold within a limit, mark and advance.
    // Also track "slightly stuck" if release takes >2s (still succeeds).
    if (_calibOnStartMs == 0) _calibOnStartMs = millis();
    const uint32_t slightlyStuckMs = 1000;
    const uint32_t notReleasingMs = 6000;
    if (millis() - _calibOnStartMs > notReleasingMs) {
        _calibNotReleasing[_currentCalibrationKey] = true;
        if (_outputFormat) {
          int board, boardKey;
          getBoardAndBoardKey(_currentCalibrationKey, board, boardKey);
          Serial.print("{\"type\":\"calib_row\",\"key\":");
          Serial.print(_currentCalibrationKey);
          Serial.print(",\"board\":");
          Serial.print(board);
          Serial.print(",\"boardKey\":");
          Serial.print(boardKey);
          Serial.print(",\"ok\":0,\"reason\":\"NotReleasing\",");
          Serial.print("\"maxSwing\":");
          Serial.print(_maxSwing[_currentCalibrationKey], 6);
          Serial.print(",\"pol\":");
          Serial.print((int)_polarization[_currentCalibrationKey]);
          Serial.println("}");
        } else {
          _println("calib NotReleasing key=%d maxSwing=%.4f", _currentCalibrationKey + 1, _maxSwing[_currentCalibrationKey]);
        }

        setColor(_currentCalibrationKey, 255, 0, 0);
        updateAllLEDs();
        delay(250);
        setColor(_currentCalibrationKey, 0, 0, 0);
        updateAllLEDs();

        _calibOnStartMs = 0;
        if (_currentCalibrationKey < _toKey) {
            _currentCalibrationKey++;
            _calibNeedSettle = true;
            _calibSettleStartMs = millis();
            _calibLoopState = STATE_CALIBRATION_OFF;
        } else {
            _calibLoopState = STATE_CALIBRATION_STOP;
        }
        return;
    }

    if (swing < _threshold_delta[_currentCalibrationKey]) {
        const uint32_t releaseMs = millis() - _calibOnStartMs;
        if (releaseMs > slightlyStuckMs) {
          _calibSlightlyStuck[_currentCalibrationKey] = true;
        }
        _calibReleaseMs[_currentCalibrationKey] = releaseMs;

        // Successful calibration for this key.
        _hasZero[_currentCalibrationKey] = true;
        float percentageFull = swing / MAX_VOLTAGE * 100;
        _println("[%d] Under threshold: %f V. Max swing: %f V (%f \%) Polarization: %d", 
            _currentCalibrationKey+1, 
            currentVoltage,
            _maxSwing[_currentCalibrationKey],
            percentageFull,
            _polarization[_currentCalibrationKey]);

        // Emit per-key row immediately
        if (_outputFormat) {
          int board, boardKey;
          getBoardAndBoardKey(_currentCalibrationKey, board, boardKey);
          Serial.print("{\"type\":\"calib_row\",\"key\":");
          Serial.print(_currentCalibrationKey);
          Serial.print(",\"board\":");
          Serial.print(board);
          Serial.print(",\"boardKey\":");
          Serial.print(boardKey);
          Serial.print(",\"ok\":1");
          Serial.print(",\"baseline\":");
          Serial.print(_zeroVoltage[_currentCalibrationKey], 6);
          Serial.print(",\"sigma\":");
          Serial.print(_noiseUnscaled[_currentCalibrationKey], 6);
          Serial.print(",\"thr\":");
          Serial.print(_threshold_delta[_currentCalibrationKey], 6);
          Serial.print(",\"maxSwing\":");
          Serial.print(_maxSwing[_currentCalibrationKey], 6);
          Serial.print(",\"pol\":");
          Serial.print((int)_polarization[_currentCalibrationKey]);
          Serial.print(",\"releaseMs\":");
          Serial.print(releaseMs);
          Serial.print(",\"slightlyStuck\":");
          Serial.print(releaseMs > slightlyStuckMs ? 1 : 0);
          Serial.println("}");
        }

        // LED: green = stored
        setColor(_currentCalibrationKey, 0, 255, 0);
        updateAllLEDs();
        delay(150);
        setColor(_currentCalibrationKey, 0, 0, 0);
        updateAllLEDs();
        _calibOnStartMs = 0;
        if (_currentCalibrationKey < _toKey) {
            _currentCalibrationKey++;
            _calibNeedSettle = true;
            _calibSettleStartMs = millis();
            _calibLoopState = STATE_CALIBRATION_OFF;
        } else
            _calibLoopState = STATE_CALIBRATION_STOP;
    }
  }
  
void loopCalibrationStop() {
    analogReadAveraging(_measureAvgStandard);
    saveCalibrationCSV();
    _calibDirty = false;
    updateAllLEDs();

    // Persist last-run issues to SD for post-processing.
    {
      const char* metaFile = "calib_meta.csv";
      SD.remove(metaFile);
      File f = SD.open(metaFile, FILE_WRITE);
      if (f) {
        f.println("#calib_meta_v1");
        f.println("#key,board,boardKey,skipped,notTriggering,selfTriggering,notReleasing,slightlyStuck,releaseMs");
        for (int k = _fromKey; k <= _toKey; ++k) {
          int board, boardKey;
          getBoardAndBoardKey(k, board, boardKey);
          f.print(k);
          f.print(',');
          f.print(board);
          f.print(',');
          f.print(boardKey);
          f.print(',');
          f.print(_calibSkipped[k] ? 1 : 0);
          f.print(',');
          f.print(_calibFailed[k] ? 1 : 0);
          f.print(',');
          f.print(_calibSelfTriggering[k] ? 1 : 0);
          f.print(',');
          f.print(_calibNotReleasing[k] ? 1 : 0);
          f.print(',');
          f.print(_calibSlightlyStuck[k] ? 1 : 0);
          f.print(',');
          f.println((unsigned long)_calibReleaseMs[k]);
        }
        f.close();
      }
    }

    _calibRunHasSummary = true;

    if (_outputFormat) {
      Serial.println("{\"type\":\"calib_saved\",\"file\":\"calib.csv\",\"ok\":1}");

      // Summarize issues for the selected range
      int skipped = 0;
      int notTriggering = 0;
      int selfTriggering = 0;
      int notReleasing = 0;
      int slightlyStuck = 0;
      for (int k = _fromKey; k <= _toKey; ++k) {
        if (_calibSkipped[k]) skipped++;
        if (_calibFailed[k]) notTriggering++;
        if (_calibSelfTriggering[k]) selfTriggering++;
        if (_calibNotReleasing[k]) notReleasing++;
        if (_calibSlightlyStuck[k]) slightlyStuck++;
      }
      Serial.print("{\"type\":\"calib_done\",\"Skipped\":");
      Serial.print(skipped);
      Serial.print(",\"NotTriggering\":");
      Serial.print(notTriggering);
      Serial.print(",\"SelfTriggering\":");
      Serial.print(selfTriggering);
      Serial.print(",\"NotReleasing\":");
      Serial.print(notReleasing);
      Serial.print(",\"SlightlyStuck\":");
      Serial.print(slightlyStuck);
      Serial.println("}");

      for (int k = _fromKey; k <= _toKey; ++k) {
        if (!_calibSkipped[k] && !_calibFailed[k] && !_calibSelfTriggering[k] && !_calibNotReleasing[k] && !_calibSlightlyStuck[k]) continue;
        int board, boardKey;
        getBoardAndBoardKey(k, board, boardKey);
        Serial.print("{\"type\":\"calib_issue\",\"key\":");
        Serial.print(k);
        Serial.print(",\"board\":");
        Serial.print(board);
        Serial.print(",\"boardKey\":");
        Serial.print(boardKey);
        Serial.print(",\"reason\":\"");
        if (_calibSelfTriggering[k]) Serial.print("SelfTriggering");
        else if (_calibNotReleasing[k]) Serial.print("NotReleasing");
        else if (_calibFailed[k]) Serial.print("NotTriggering");
        else if (_calibSkipped[k]) Serial.print("Skipped");
        else Serial.print("SlightlyStuck");
        if (_calibSlightlyStuck[k]) {
          Serial.print("\",\"releaseMs\":");
          Serial.print(_calibReleaseMs[k]);
          Serial.print(",\"slightlyStuck\":1");
          Serial.println("}");
          continue;
        }
        Serial.println("\"}");
      }
    } else {
      int skipped = 0;
      int notTriggering = 0;
      int selfTriggering = 0;
      int notReleasing = 0;
      for (int k = _fromKey; k <= _toKey; ++k) {
        if (_calibSkipped[k]) skipped++;
        if (_calibFailed[k]) notTriggering++;
        if (_calibSelfTriggering[k]) selfTriggering++;
        if (_calibNotReleasing[k]) notReleasing++;
      }
      _println("calib done Skipped=%d NotTriggering=%d SelfTriggering=%d NotReleasing=%d", skipped, notTriggering, selfTriggering, notReleasing);
      for (int k = _fromKey; k <= _toKey; ++k) {
        if (!_calibSkipped[k] && !_calibFailed[k] && !_calibSelfTriggering[k] && !_calibNotReleasing[k]) continue;
        int board, boardKey;
        getBoardAndBoardKey(k, board, boardKey);
        const char *reason = _calibSelfTriggering[k] ? "SelfTriggering" : (_calibNotReleasing[k] ? "NotReleasing" : (_calibFailed[k] ? "NotTriggering" : "Skipped"));
        _println("calib issue key=%d board=%d keyOnBoard=%d reason=%s", k + 1, board + 1, boardKey + 1, reason);
      }
    }
  
    _calibLoopState = STATE_RUNNING;
  }

void printLastCalibrationIssues()
{
  // Prints the last calibration run issues for the currently selected range.
  if (!_calibRunHasSummary) {
    if (_outputFormat) {
      Serial.println("{\"type\":\"calissues\",\"ok\":0,\"error\":\"no_summary\"}");
    } else {
      _println("calissues: no completed calibration summary yet");
    }
    return;
  }

  int skipped = 0;
  int notTriggering = 0;
  int selfTriggering = 0;
  int notReleasing = 0;
  int slightlyStuck = 0;
  for (int k = _fromKey; k <= _toKey; ++k) {
    if (_calibSkipped[k]) skipped++;
    if (_calibFailed[k]) notTriggering++;
    if (_calibSelfTriggering[k]) selfTriggering++;
    if (_calibNotReleasing[k]) notReleasing++;
    if (_calibSlightlyStuck[k]) slightlyStuck++;
  }

  if (_outputFormat) {
    Serial.print("{\"type\":\"calissues\",\"ok\":1,\"from\":");
    Serial.print(_fromKey);
    Serial.print(",\"to\":");
    Serial.print(_toKey);
    Serial.print(",\"Skipped\":");
    Serial.print(skipped);
    Serial.print(",\"NotTriggering\":");
    Serial.print(notTriggering);
    Serial.print(",\"SelfTriggering\":");
    Serial.print(selfTriggering);
    Serial.print(",\"NotReleasing\":");
    Serial.print(notReleasing);
    Serial.print(",\"SlightlyStuck\":");
    Serial.print(slightlyStuck);
    Serial.println("}");

    for (int k = _fromKey; k <= _toKey; ++k) {
      if (!_calibSkipped[k] && !_calibFailed[k] && !_calibSelfTriggering[k] && !_calibNotReleasing[k] && !_calibSlightlyStuck[k]) continue;
      int board, boardKey;
      getBoardAndBoardKey(k, board, boardKey);
      Serial.print("{\"type\":\"calissue_row\",\"key\":");
      Serial.print(k);
      Serial.print(",\"board\":");
      Serial.print(board);
      Serial.print(",\"boardKey\":");
      Serial.print(boardKey);
      Serial.print(",\"skipped\":");
      Serial.print(_calibSkipped[k] ? 1 : 0);
      Serial.print(",\"notTriggering\":");
      Serial.print(_calibFailed[k] ? 1 : 0);
      Serial.print(",\"selfTriggering\":");
      Serial.print(_calibSelfTriggering[k] ? 1 : 0);
      Serial.print(",\"notReleasing\":");
      Serial.print(_calibNotReleasing[k] ? 1 : 0);
      Serial.print(",\"slightlyStuck\":");
      Serial.print(_calibSlightlyStuck[k] ? 1 : 0);
      Serial.print(",\"releaseMs\":");
      Serial.print((unsigned long)_calibReleaseMs[k]);
      Serial.println("}");
    }
  } else {
    _println("calissues from=%d to=%d Skipped=%d NotTriggering=%d SelfTriggering=%d NotReleasing=%d SlightlyStuck=%d",
             _fromKey + 1, _toKey + 1,
             skipped, notTriggering, selfTriggering, notReleasing, slightlyStuck);
  }
}
  
  
void saveCalibrationCSV() {
    char filename[255];
    sprintf(filename, "calib.csv");
    Serial.println(filename);
    SD.remove(filename);
    File configFile = SD.open(filename, FILE_WRITE);
    if (configFile) {
        configFile.println("#calib_v3");
        Serial.println("Saving calibration to CSV...");
        for (int i = 0; i < NUM_KEYS; i++) {
          configFile.print(i);
          configFile.print(",");
          configFile.print(_hasZero[i] ? 1 : 0);
          configFile.print(",");
          configFile.print(_maxSwing[i]);          
          configFile.print(",");
          configFile.print(_polarization[i]);
          configFile.print(",");
          configFile.print(_noiseUnscaled[i], 6);
          configFile.print(",");
          configFile.print(_threshold_delta[i], 6);
          configFile.print(",");
          configFile.println(_zeroVoltage[i], 6);
        }
        configFile.close();
        Serial.println("CSV calibration saved.");
    } else {
        Serial.println("Error opening calibration file for writing.");
    }
}

bool loadCalibrationCSV() {
    // Always start from a clean slate before applying persisted calibration.
    clearCalibration(0, NUM_KEYS - 1);

    // Open "calib.csv" from the SD card
    File configFile = SD.open("calib.csv", FILE_READ);
    if (!configFile) {
        Serial.println("Error opening calibration file for reading.");
        return false;
    }

    Serial.println("Loading calibration from CSV...");

    // Detect version / format. Treat unknown or old formats as "no calibration".
    String firstLine;
    while (configFile.available()) {
        firstLine = configFile.readStringUntil('\n');
        firstLine.trim();
        if (firstLine.length() == 0) continue;
        break;
    }

    const bool isV3 = firstLine.startsWith("#calib_v3");
    if (!isV3) {
        // Old format or unknown file; behave as if no calibration exists.
        configFile.close();
        clearCalibration(0, NUM_KEYS - 1);
        Serial.println("Calibration file missing/old format. Starting uncalibrated.");
        Serial.println("Select range with k<from>,<to> then run x to calibrate.");
        return false;
    }

    // We'll mark clean only if we successfully parse at least one row.
    bool any = false;

    // Read the file until no more lines
    while (configFile.available()) {
        String line = configFile.readStringUntil('\n');
        line.trim();  // Remove any trailing whitespace or newline
        if (line.length() == 0) {
            continue;  // Skip empty lines
        }

        if (line.startsWith("#")) {
            continue; // skip comments/header
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

        any = true;

        // 2) hasZero
        nextPos = line.indexOf(',', lastPos);
        if (nextPos < 0) {
            continue;
        }
        const int hasZero = line.substring(lastPos, nextPos).toInt();
        lastPos = nextPos + 1;

        // Next: _maxSwing[key]
        nextPos = line.indexOf(',', lastPos);
        if (nextPos < 0) {
            continue;
        }
        _maxSwing[key] = line.substring(lastPos, nextPos).toFloat();
        lastPos = nextPos + 1;

        // 3) _polarization[key]
        nextPos = line.indexOf(',', lastPos);
        if (nextPos < 0) {
            continue;
        }
        _polarization[key] = line.substring(lastPos, nextPos).toInt();
        lastPos = nextPos + 1;

        // 4) _noiseUnscaled[key]
        nextPos = line.indexOf(',', lastPos);
        if (nextPos < 0) {
            continue;
        }
        _noiseUnscaled[key] = line.substring(lastPos, nextPos).toFloat();
        lastPos = nextPos + 1;

        // 5) _threshold_delta[key]
        nextPos = line.indexOf(',', lastPos);
        if (nextPos < 0) {
            continue;
        }
        _threshold_delta[key] = line.substring(lastPos, nextPos).toFloat();
        lastPos = nextPos + 1;

        // 7) _zeroVoltage[key] (last field)
        _zeroVoltage[key] = line.substring(lastPos).toFloat();

        _hasZero[key] = (hasZero != 0);

        // If the row is not calibrated, force all fields to defaults.
        if (!_hasZero[key]) {
          _zeroVoltage[key] = 0.0f;
          _noiseUnscaled[key] = 0.0f;
          _threshold_delta[key] = 0.0f;
          _maxSwing[key] = 0.0f;
          _polarization[key] = 0;
        }
    }

    configFile.close();
    Serial.println("CSV calibration loaded.");
    _calibDirty = !any ? true : false;
    return true;
}


// PEAK DETECT
void peakDetect(float voltage, int key) {
    if (_diagActive) return;
    int board, boardKey;
    // "static" variables keep their numbers between each run of this function
    static int state[NUM_KEYS];  // 0=idle, 1=looking for peak, 2=ignore aftershocks
    static float peak[NUM_KEYS];   // remember the highest reading
    static elapsedMillis msec[NUM_KEYS]; // timer to end states 1 and 2
    static bool playing[NUM_KEYS];
    // Expose activity state for baseline drift tracker.
    // 0=idle,1=peak,2=aftershock in this state machine.
    // Update activity flag for this key.
    _baselineKeyActive[key] = (state[key] != 0);
  
    if (!_hasZero[key]) return;

    float voltageSwing = fabs(_zeroVoltage[key] - voltage); 
    const float thr = getEffectiveThresholdDelta(key);
    
    switch (state[key]) {
      // IDLE state: wait for any reading is above threshold.  Do not set
      // the threshold too low.  You don't want to be too sensitive to slight
      // vibration.
      case 0:
        if (voltageSwing > thr) {
            if (_debugMode) {
              if (_outputFormat) {
                beginJson("peak_begin");
                printJsonKV("key", key);
                printJsonKV("v", voltage);
                printJsonKV("zero", _zeroVoltage[key]);
                printJsonKV("swing", voltageSwing);
                printJsonKV("thr", thr, true);
                endJson();
              } else {
                _print("BEGIN [");
                _print(key);
                _print("] ");
                _print(voltage);
                _print(" OFF: ");
                _println(_zeroVoltage[key]);
              }
            }
  
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
          int velocity = map(peak[key], thr, _maxSwing[key], 1, 127);
          if (_debugMode) {
            if (_outputFormat) {
              beginJson("peak_fire");
              printJsonKV("key", key);
              printJsonKV("peak", peak[key]);
              printJsonKV("thr", thr);
              printJsonKV("maxSwing", _maxSwing[key]);
              printJsonKV("vel", velocity);
              printJsonKV("kind", "peak", true);
              endJson();
            } else {
              _println("PEAK [%d] %f %d", key+1, peak[key], velocity);
            }
          }
          if (velocity > 127) {
            const float oldMax = _maxSwing[key];
            _maxSwing[key] = peak[key];
            velocity = map(peak[key], thr, _maxSwing[key], 1, 127);
            if (_debugMode) {
              if (_outputFormat) {
                beginJson("peak_clamp");
                printJsonKV("key", key);
                printJsonKV("zero", _zeroVoltage[key]);
                printJsonKV("oldMaxSwing", oldMax);
                printJsonKV("newMaxSwing", _maxSwing[key], true);
                endJson();
              } else {
                _println("CAL [%d] Off=%f On=%f", key+1, _zeroVoltage[key], _maxSwing[key]);
              }
            }
            if (velocity > 127) {
              if (_debugMode) {
                if (_outputFormat) {
                  beginJson("peak_error");
                  printJsonKV("key", key);
                  printJsonKV("error", "vel_gt_127", true);
                  endJson();
                } else {
                  _println("ERROR: Velocity higher than 127, better investigate!");
                }
              }
              velocity = 127;
            }
          }
          if (!playing[key]) {
            usbMIDI.sendNoteOn(_fields[board][boardKey].Note, velocity, _fields[board][boardKey].Channel);
            playing[key] = true;     
            LEDStrip->setPixelColor(key, 255, 255, 255);
            _baselineCooldownMs = 0;
            if (_debugMode && _outputFormat) {
              beginJson("midi");
              printJsonKV("kind", "noteon");
              printJsonKV("key", key);
              printJsonKV("note", _fields[board][boardKey].Note);
              printJsonKV("ch", _fields[board][boardKey].Channel);
              printJsonKV("vel", velocity, true);
              endJson();
            }
          } else {
            usbMIDI.sendPolyPressure(_fields[board][boardKey].Note, velocity, _fields[board][boardKey].Channel); // Send aftertouch data
            _baselineCooldownMs = 0;
            if (_debugMode && _outputFormat) {
              beginJson("midi");
              printJsonKV("kind", "polypressure");
              printJsonKV("key", key);
              printJsonKV("note", _fields[board][boardKey].Note);
              printJsonKV("ch", _fields[board][boardKey].Channel);
              printJsonKV("val", velocity, true);
              endJson();
            }
          }
          msec[key] = 0;
          state[key] = 2;
        }
        return;
  
      // Ignore Aftershock state: wait for things to be quiet azeroPoint.
      default:
        if (voltageSwing > thr) {
            msec[key] = 0; // keep resetting timer if above threshold
            state[key] = 1; 
        } else if (msec[key] > _aftershockMillis) {
            getBoardAndBoardKey(key, board, boardKey);
            usbMIDI.sendNoteOff(_fields[board][boardKey].Note, 0, _fields[board][boardKey].Channel);
            if (_debugMode && _outputFormat) {
              beginJson("midi");
              printJsonKV("kind", "noteoff");
              printJsonKV("key", key);
              printJsonKV("note", _fields[board][boardKey].Note);
              printJsonKV("ch", _fields[board][boardKey].Channel, true);
              endJson();
            }
            updateLED(key);
            playing[key] = false;
            state[key] = 0; // go back to idle when
        }
    }
  }
