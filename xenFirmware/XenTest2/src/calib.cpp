#include "xen.h"
#include "calib.h"
#include "midi.h"
#include "utils.h"
#include "mux.h"
#include "adc.h"
#include "dac.h"
#include "sdcard.h"
#include "led.h"

float _threshold_delta[NUM_KEYS];
float _zeroVoltage[NUM_KEYS];

int _currentCalibrationKey = 0;
int _calibLoopState = STATE_CALIBRATION_START;
int _gain[NUM_KEYS];
int _offset[NUM_KEYS];
float _maxSwing[NUM_KEYS];
short _polarization[NUM_KEYS];
bool _manualCalibration = false;
float _noiseUnscaled[NUM_KEYS];
float _noiseScaled[NUM_KEYS];

int _measureAvgStandard = 32;
int _measureAvgCalibration = 32;

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

static inline void scanKey(int key)
{   // do not forget to set DAC and dPot for this key before
    const int  adc = getAdcMain(key);
    const float v  = getAdcVoltage(adc);
    peakDetect(v, key);
}

void scanKeysNormal() {
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
    
      if (doA)
        setDacMain(keyA, _offset[keyA]);
      if (doB)
        setDacMain(keyB, _offset[keyB]);

      setKey(muxPos);                     // one change of the analogue multiplexer
      delayMicroseconds(_us_delay_after_dac_set - _us_delay_after_mux);

      if (doA) scanKey(keyA);
      if (doB) scanKey(keyB);
  }
}

void scanNoise(int ms) {    
    // --- per-key running statistics (Welford) -------------------------------
    static float meanScaled  [NUM_KEYS];
    static float m2Scaled    [NUM_KEYS];
    static float meanUnscaled[NUM_KEYS];
    static float m2Unscaled  [NUM_KEYS];
    static uint32_t n[NUM_KEYS];

    memset(meanScaled,   0, sizeof(meanScaled));
    memset(m2Scaled,     0, sizeof(m2Scaled));
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

            if (doA) setDacMain(keyA, _offset[keyA]);
            if (doB) setDacMain(keyB, _offset[keyB]);

            setKey(muxPos);
            delayMicroseconds(_us_delay_after_dac_set - _us_delay_after_mux);

            // ------------------------------- sample A ----------------------
            if (doA)
            {
                const float vMain = getAdcVoltage(getAdcMain(keyA));
                const float vRaw  = getAdcVoltage(getAdcUnscaled(keyA));

                ++n[keyA];

                // --- Main path (scaled) ---
                float delta = vMain - meanScaled[keyA];
                meanScaled[keyA] += delta / n[keyA];
                m2Scaled[keyA]   += delta * (vMain - meanScaled[keyA]);

                // --- Raw path (unscaled) ---
                delta = vRaw - meanUnscaled[keyA];
                meanUnscaled[keyA] += delta / n[keyA];
                m2Unscaled[keyA]   += delta * (vRaw - meanUnscaled[keyA]);
            }

            // ------------------------------- sample B ----------------------
            if (doB)
            {
                const float vMain = getAdcVoltage(getAdcMain(keyB));
                const float vRaw  = getAdcVoltage(getAdcUnscaled(keyB));

                ++n[keyB];

                float delta = vMain - meanScaled[keyB];
                meanScaled[keyB] += delta / n[keyB];
                m2Scaled[keyB]   += delta * (vMain - meanScaled[keyB]);

                delta = vRaw - meanUnscaled[keyB];
                meanUnscaled[keyB] += delta / n[keyB];
                m2Unscaled[keyB]   += delta * (vRaw - meanUnscaled[keyB]);
            }
        } // for muxPos
    }     // while time

    // ----------- convert M2 to σ and store in global output arrays ----------
    for (int k = 0; k < NUM_KEYS; ++k)
    {
        if (n[k] > 1) {
            _noiseScaled[k]   = sqrtf(m2Scaled[k]   / (n[k] - 1));  // σMAIN
            _noiseUnscaled[k] = sqrtf(m2Unscaled[k] / (n[k] - 1));  // σRAW
        } else {
            _noiseScaled[k]   = 0.0f;
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

        float maxScaled   = 0.0f, maxUnscaled   = 0.0f;
        float sumScaled   = 0.0f, sumUnscaled   = 0.0f;
        int   cntScaled   = 0   , cntUnscaled   = 0   ;

        for (int k = first; k <= last; ++k)
        {
            const float ns = _noiseScaled[k];
            const float nu = _noiseUnscaled[k];

            if (ns > maxScaled)   maxScaled   = ns;
            if (nu > maxUnscaled) maxUnscaled = nu;

            if (ns > 0.0f) { sumScaled   += ns; ++cntScaled;   }
            if (nu > 0.0f) { sumUnscaled += nu; ++cntUnscaled; }
        }

        const float avgScaled   = cntScaled   ? sumScaled   / cntScaled   : 0.0f;
        const float avgUnscaled = cntUnscaled ? sumUnscaled / cntUnscaled : 0.0f;

        _println("Keys %3d-%3d  |  Main σ ≤ %.6f  (σ̄ = %.6f)  |  Unscaled σ ≤ %.6f  (σ̄ = %.6f)",
                 first + 1, last + 1,
                 maxScaled,   avgScaled,
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
//      adc == 0  → σ measured on the *Main* path  (_noiseScaled)
//      adc == 1  → σ measured on the *Unscaled* path  (_noiseUnscaled)
//---------------------------------------------------------------------------
void showNoise(int adc)
{
    const float *noise = (adc == 0) ? _noiseScaled : _noiseUnscaled;

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
    _println("[%d] Threshold=%f V", key+1, _threshold_delta[key]);
}

void setOffset(int key, int offset) {
    _offset[key] = offset;
}

void setGain(int key, int gain) {
    _gain[key] = gain;
}

void setThreshold(int key, float threshold) {
    _threshold_delta[key]= threshold;
}

void loopCalibrationStart() {
    bool ok = !_manualCalibration && loadCalibrationCSV();
    
    if (ok) { 
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
  
    if (swing > _threshold_delta[_currentCalibrationKey]) {
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
  
    if (swing < _threshold_delta[_currentCalibrationKey]) {
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
          configFile.print(_polarization[i]);
          configFile.print(",");
          configFile.print(_noiseScaled[i]);
          configFile.print(",");
          configFile.println(_noiseUnscaled[i]);
          configFile.print(",");
          configFile.println(_threshold_delta[i]);
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

        _threshold_delta[key] = key <= 267 ? 0.30f : 0.40f;

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
        if (voltageSwing > _threshold_delta[key]) {
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
          int velocity = map(peak[key], _threshold_delta[key], _maxSwing[key], 1, 127);
          _println("PEAK [%d] %f %d", key+1, peak[key], velocity);
          if (velocity > 127) {
            _maxSwing[key] = peak[key];
            velocity = map(peak[key], _threshold_delta[key], _maxSwing[key], 1, 127);
            _println("CAL [%d] Off=%f On=%f", key+1, _zeroVoltage[key], _maxSwing[key]);
            if (velocity > 127) {
              _println("ERROR: Velocity higher than 127, better investigate!");
              velocity = 127;
            }
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
        if (voltageSwing > _threshold_delta[key]) {
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