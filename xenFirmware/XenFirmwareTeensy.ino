#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <FastLED.h>
#include "MCP_DAC.h"

char _charVal[80];

// ADC
#define NUM_ADCS           4
#define ADC_MAIN           0
#define ADC_FIRSTSTAGE     1
#define ADC_UNUSED         2
#define ADC_UNSCALED       3

const short _adcPins[NUM_ADCS] = { A2, A16, A17, A15 };
const float ADC_VREF = 3.3;       // Reference voltage in volts
const int ADC_RESOLUTION = 12;
const int ADC_RESOLUTION_MAX = 4095;  // 10 bit 1023, 12 bit 4095
const int DAC_RESOLUTION = 12;
const int DAC_RESOLUTION_MAX = 4095;  // 10 bit 1023, 12 bit 4095
const int DAC_VREF = 5;


// DAC
#define MAINDAC_CS_PIN    10
#define OFFSETDAC_CS_PIN  0

// MUX
#define NUM_TOP_MUX        2
#define NUM_MUX_PER_BOARD  4
#define NUM_SELECTLINES    4
const int _selectPins[NUM_TOP_MUX][NUM_SELECTLINES] = { {2, 3, 4, 5}, {6, 7, 8, 9} }; // Change these pin numbers based on your actual wiring
const int _keysPerMux[NUM_MUX_PER_BOARD] = { 15, 15, 15, 11 };
const int _muxPerTopMux[NUM_TOP_MUX] = { 12, 12 };

// XEN

#define NUM_BOARDS 5
#define NUM_KEYS 112
#define NUM_KEYS_PER_BOARD 56

typedef struct {
    byte Board;
    byte BoardKey;
    byte Channel;
    byte Note;
    CRGB Color;     // Pin number such as GPIO_PIN_3, GPIO_PIN_5, etc.
} XenField;

XenField _fields[NUM_BOARDS][NUM_KEYS_PER_BOARD];
byte _program = 0;

// LED

#define DATA_PIN 17

CRGB _leds[NUM_KEYS];

// SD-CARD

File _myFile;
const int _sdCardChipSelect = BUILTIN_SDCARD;

// MIDI OUTPUT
#define PLOT_OUTPUT       0
#define BASE_NOTE         48

#define STATE_UNCALIB     0
#define STATE_CALIB       1

#define MAX_VOLTAGE          3.3
#define THRESHOLD_DELTA      0.25
#define STABILITY_THRESHOLD  0.05

const int _peakTrackMillis = 12;
const int _aftershockMillis = 25; // aftershocks & vibration reject

int _calibrated[NUM_KEYS];
float _zeroVoltage[NUM_KEYS];

// CALIB
#define FIXED_GAIN 9.75

#define STATE_RUNNING 0
#define STATE_CALIBRATION_START 1
#define STATE_CALIBRATION_OFF 2
#define STATE_CALIBRATION_WAIT_ON 3
#define STATE_CALIBRATION_ON 4
#define STATE_CALIBRATION_STOP 5

int _currentCalibrationKey = 0;
int _loopState = STATE_CALIBRATION_START;
int _gain[NUM_KEYS];
int _offset[NUM_KEYS];
float _maxSwing[NUM_KEYS];
short _polarization[NUM_KEYS];

// SYSEX AND MTS

const int MTS_SYSEX_SIZE = 408;  // Expected size of the .syx file
const int MTS_NAME_OFFSET = 6;   // Byte index where the tuning name starts
const int MTS_NAME_LENGTH = 16;  // Length of the tuning name
byte _currentTuning[MTS_SYSEX_SIZE];
bool _tuningLoaded = false;

// FUNCTIONS

void setup() {
  Serial.begin(115200);

  setupCard();

  setupMux();
  setupADC();
  setupDAC();

  setupLEDs();
  setupMidi();

  loadConfigurationCSV();
  mts_loadTuning();
  mts_sendTuning();
}

void setupCard() {
   Serial.println("Initializing SD card...");

  if (!SD.begin(_sdCardChipSelect)) {
    Serial.println("SD card init failed!");
    return;
  }
  Serial.println("SD card initialized.");
}

void printBytes(const byte *data, unsigned int size) {
  while (size > 0) {
    byte b = *data++;
    if (b < 16) Serial.print('0');
    Serial.print(b, HEX);
    if (size > 1) Serial.print(' ');
    size = size - 1;
  }
}

byte mergeBytes(byte low, byte high) {
    return (low & 0x7F) | ((high & 0x7F) << 7);
}

void setupLEDs() {
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(_leds, NUM_KEYS);  

  // Turn the LED on, then pause
  for (int i=0; i < NUM_KEYS; i++)
    _leds[i] = CRGB::Blue;
  FastLED.show();
}

void updateLEDs() {
  Serial.println("Updating LEDs...");
  // Turn the LED on, then pause
  int l = 0;
  for (int b = 0; b < NUM_BOARDS; b++) {
    for (int c = 0; c < NUM_KEYS_PER_BOARD; c++) {
      _leds[l++] = _fields[b][c].Color;
    }
    if (l >= NUM_KEYS) break;
  }
  FastLED.show();
  Serial.println("LEDs updated.");
}


void loop() {
  usbMIDI.read();

  if (_loopState == STATE_CALIBRATION_START) 
    loopCalibrationStart();
  else if (_loopState == STATE_CALIBRATION_OFF) 
    loopCalibrationOff();
  else if (_loopState == STATE_CALIBRATION_WAIT_ON) 
    loopCalibrationWaitOn();
  else if (_loopState == STATE_CALIBRATION_ON) 
    loopCalibrationOn();
  else if (_loopState == STATE_CALIBRATION_STOP) 
    loopCalibrationStop();
  else if (_loopState == STATE_RUNNING)
    loopNormal();
}

void loopNormal() {
  if (PLOT_OUTPUT == 1) { 
    Serial.print("Min:");
    Serial.print(0);
  }
  for (int key=0; key<NUM_KEYS; key++) {
    setKey(key);
    mainDAC(_offset[key], 0); // set zeroPoint to center  
    delayMicroseconds(30);

    int analogValue = analogRead(_adcPins[ADC_MAIN]);
    float voltage = getAdcVoltage(analogValue); // (analogValue * VREF) / ADC_RESOLUTION; 
    peakDetect(voltage, key);

    if (PLOT_OUTPUT == 1) { 
      sprintf(_charVal, ",Key%d:", key);
      Serial.print(_charVal);
      Serial.print(voltage);
    }
  }
  if (PLOT_OUTPUT == 1) { 
    Serial.print(",Max:");
    Serial.println(MAX_VOLTAGE);
    delay(10);
  }
}

// MIDI INCOMING

void midiHandleSystemEx(const byte *data, uint16_t length, bool last) {
  Serial.print("SysEx Message: ");
  printBytes(data, length);
  if (last) {
    Serial.println(" (end)");

    int command = (int8_t)(data[2]) % 112;
    if (command == 0) 
        sysEx_saveAndUpdate();
    else if (command == 1)
        sysEx_setField(data, length);
    else if (command == 2)
        mts_receiveTuning(data, length);
  } else {
    Serial.println(" (to be continued)");
  }
}

void midiHandleProgramChange(byte channel, byte program) {
  _program = program;

  Serial.print("Program change: ");
  Serial.print(_program);
  Serial.println();

  loadConfigurationCSV();
  updateLEDs();

  _tuningLoaded = false;
  mts_loadTuning();
  mts_sendTuning();
}


void mts_receiveTuning(const byte *data, uint16_t length) {
  // data start with byte 3
  if (length != MTS_SYSEX_SIZE + 3) {
    Serial.print("Error: Expected to receive MTS data of size ");
    Serial.print(MTS_SYSEX_SIZE);
    Serial.print(" bytes, but received ");
    Serial.print(length - 3);
    Serial.println(" bytes.");
    return;
  }

  // Copy the tuning data from the incoming SysEx message (excluding metadata)
  memcpy(_currentTuning, data + 3, MTS_SYSEX_SIZE);

  Serial.print("Tuning data received.");
  _tuningLoaded = true;
  mts_printTuningName();
}

bool mts_loadTuning() {
    char filename[255];
    sprintf(filename, "tuning%d.syx", _program);
    Serial.println(filename);
    File file = SD.open(filename, FILE_READ);
    if (!file) {
        Serial.println(filename);
        Serial.println("Error: Could not open tuning file!");
        return false;
    }

    // Check the actual file size before reading
    int fileSize = file.size();
    if (fileSize > MTS_SYSEX_SIZE) {
        Serial.print("Error: File size (");
        Serial.print(fileSize);
        Serial.println(" bytes) exceeds buffer size!");
        file.close();
        return false;
    }

    int bytesRead = file.read(_currentTuning, MTS_SYSEX_SIZE);
    file.close();

    if (bytesRead != MTS_SYSEX_SIZE) {
        Serial.print("Error: Expected ");
        Serial.print(MTS_SYSEX_SIZE);
        Serial.print(" bytes, but read ");
        Serial.print(bytesRead);
        Serial.println(" bytes.");
        return false;
    }

    _tuningLoaded = true;
    Serial.println("Tuning loaded.");
    mts_printTuningName();
    return true;
}

void mts_printTuningName() {
    if (!_tuningLoaded) return;
    char tuningName[MTS_NAME_LENGTH + 1];  // 16 characters + null terminator
    memset(tuningName, 0, sizeof(tuningName));

    for (int i = 0; i < MTS_NAME_LENGTH; i++) {
        if (_currentTuning[MTS_NAME_OFFSET + i] == 0x00) break;  // Stop at first null byte
        tuningName[i] = _currentTuning[MTS_NAME_OFFSET + i];
    }

    Serial.print("Tuning Name: ");
    Serial.println(tuningName);
}

void mts_sendTuning() {
  if (!_tuningLoaded) return;
  Serial.println("Sending MTS SysEx message...");
  mts_printTuningName();
  usbMIDI.sendSysEx(MTS_SYSEX_SIZE, _currentTuning, true);
  Serial.println("Tuning sent!");
}

bool mts_saveTuning() {  
    if (!_tuningLoaded) return false;  
    char filename[255];
    sprintf(filename, "tuning%d.syx", _program);
    Serial.println(filename);

    File file = SD.open(filename, FILE_WRITE);
    if (!file) {
        Serial.println("Error: Could not open file for writing!");
        return false;
    }

    int bytesWritten = file.write(_currentTuning, MTS_SYSEX_SIZE);
    file.close();

    if (bytesWritten != MTS_SYSEX_SIZE) {
        Serial.print("Error: Expected to write ");
        Serial.print(MTS_SYSEX_SIZE);
        Serial.print(" bytes, but wrote ");
        Serial.print(bytesWritten);
        Serial.println(" bytes.");
        return false;
    }

    mts_printTuningName();
    Serial.println("Tuning successfully saved to SD card.");
    return true;
}

void sysEx_saveAndUpdate() {
  saveConfigurationCSV();
  updateLEDs();
}

void sysEx_setField(const byte *data, uint16_t length) {
    int board = (byte)(data[3]) % 112;
    int boardKey = (byte)(data[4]) % 112;

    if (board > NUM_BOARDS) {
      Serial.print("Board index too high: ");
      Serial.println(NUM_BOARDS);
      return;
    }
    if (boardKey > NUM_KEYS_PER_BOARD) {
      Serial.print("BoardKey index too high: ");
      Serial.println(NUM_KEYS_PER_BOARD);
      return;
    }

    int channel = (byte)(data[5]) % 112;
    int note = (byte)(data[6]) % 112;

    Serial.print("SET [");
    Serial.print(board);
    
    Serial.print("-");
    Serial.print(boardKey);
    Serial.print("] ");
    
    _fields[board][boardKey].Board = board;
    _fields[board][boardKey].BoardKey = boardKey;
    _fields[board][boardKey].Channel = channel;
    _fields[board][boardKey].Note =  note;
    _fields[board][boardKey].Color.red = mergeBytes(data[9], data[10]);
    _fields[board][boardKey].Color.green = mergeBytes(data[11], data[12]);
    _fields[board][boardKey].Color.blue = mergeBytes(data[7], data[8]);

    Serial.print(" ");
    Serial.print(note);
    Serial.print("@");
    Serial.print(channel);
    
    Serial.print(" #");
    printBytes(_fields[board][boardKey].Color.raw, 3);
    Serial.println();
}


void saveConfigurationCSV() {
    char filename[255];
    sprintf(filename, "config%d.csv", _program);
    Serial.println(filename);
    SD.remove(filename);
    File configFile = SD.open(filename, FILE_WRITE);
    if (configFile) {
        Serial.println("Saving configuration to CSV...");
        for (int i = 0; i < NUM_BOARDS; i++) {
            for (int j = 0; j < NUM_KEYS_PER_BOARD; j++) {
                XenField field = _fields[i][j];
                configFile.print(field.Board);
                configFile.print(",");
                configFile.print(field.BoardKey);
                configFile.print(",");
                configFile.print(field.Channel);
                configFile.print(",");
                configFile.print(field.Note);
                configFile.print(",");
                configFile.print(field.Color.r);
                configFile.print(",");
                configFile.print(field.Color.g);
                configFile.print(",");
                configFile.print(field.Color.b);
                configFile.println();
            }
        }
        configFile.close();
        Serial.println("CSV configuration saved.");
    } else {
        Serial.println("Error opening config file for writing.");
    }
}

void loadConfigurationCSV() {
    char filename[255];
    sprintf(filename, "config%d.csv", _program);
    Serial.println(filename);
    File configFile = SD.open(filename, FILE_READ);
    if (configFile) {
        Serial.println("Loading configuration from CSV...");
        int boardIndex = 0;
        while (configFile.available() && boardIndex < NUM_BOARDS) {
            String line = configFile.readStringUntil('\n');
            int lastPos = 0, nextPos = 0;
            XenField field;

            nextPos = line.indexOf(',', lastPos);
            field.Board = (byte)line.substring(lastPos, nextPos).toInt();
            lastPos = nextPos + 1;

            nextPos = line.indexOf(',', lastPos);
            field.BoardKey = (byte)line.substring(lastPos, nextPos).toInt();
            lastPos = nextPos + 1;

            nextPos = line.indexOf(',', lastPos);
            field.Channel = (byte)line.substring(lastPos, nextPos).toInt();
            lastPos = nextPos + 1;

            nextPos = line.indexOf(',', lastPos);
            field.Note = (byte)line.substring(lastPos, nextPos).toInt();
            lastPos = nextPos + 1;

            nextPos = line.indexOf(',', lastPos);
            field.Color.r = (byte)line.substring(lastPos, nextPos).toInt();
            lastPos = nextPos + 1;

            nextPos = line.indexOf(',', lastPos);
            field.Color.g = (byte)line.substring(lastPos, nextPos).toInt();
            lastPos = nextPos + 1;

            nextPos = line.indexOf(',', lastPos);
            field.Color.b = (byte)line.substring(lastPos, nextPos).toInt();
            
            if (field.Board >= 0 && field.Board < NUM_BOARDS && field.Board >= 0 && field.BoardKey < NUM_KEYS_PER_BOARD)
              _fields[field.Board][field.BoardKey] = field;
        }
        configFile.close();
        Serial.println("CSV configuration loaded.");
    } else {
        Serial.println("Error opening config file for reading.");
    }
}

void setupMux() { 
  for (int i = 0; i < NUM_SELECTLINES; i++) {
    pinMode(_selectPins[0][i], OUTPUT);
  }
  for (int i = 0; i < NUM_SELECTLINES; i++) {
    pinMode(_selectPins[1][i], OUTPUT);
  }
  Serial.println("MUX initialized.");
  setMux(0, 0);
}

void setMux(int selectLine0, int selectLine1) {
  for (int i = 0; i < NUM_SELECTLINES; i++) {
    digitalWriteFast(_selectPins[0][i], (selectLine0 >> i) & 1);
  }
  for (int i = 0; i < NUM_SELECTLINES; i++) {
    digitalWriteFast(_selectPins[1][i], (selectLine1 >> i) & 1);
  }
  
  if (PLOT_OUTPUT == 0) { 
    //Serial.print("MUX=");
    //Serial.print(selectLine0);
    //Serial.print("-");
    //Serial.print(selectLine1);
    //Serial.println("");
  }
  delayMicroseconds(10);
}

void setKey(int key) {
  int board, childMux, childMuxOutput, topMux, topMuxOutput;
  getMuxAndOutput(key, board, childMux, childMuxOutput);
  getTopMuxAndOutput(board, childMux, topMux, topMuxOutput);
  
  if (PLOT_OUTPUT == 0) { 
    //Serial.print("KEY ");
    //Serial.print(key);
    //Serial.print(": ");
  }
  setMux(childMuxOutput, topMuxOutput);
}

void getMuxAndOutput(int key, int &board, int &mux, int &output) {
    if (key < 0) {
        board = -1;
        mux = -1;
        output = -1;
        return;
    }

    int keysPerBoard = 0;
    for (int i = 0; i < NUM_MUX_PER_BOARD; i++) {
        keysPerBoard += _keysPerMux[i];
    }

    board = key / keysPerBoard;
    int localKey = key % keysPerBoard;

    int cumulativeKeys = 0;
    for (mux = 0; mux < NUM_MUX_PER_BOARD; mux++) {
        if (localKey < cumulativeKeys + _keysPerMux[mux]) {
            output = localKey - cumulativeKeys;
            return;
        }
        cumulativeKeys += _keysPerMux[mux];
    }

    board = -1;
    mux = -1;
    output = -1;
}


void getTopMuxAndOutput(int board, int childMux, int &topMux, int &topMuxOutput) {
    if (board < 0 || childMux < 0) {
        topMux = -1;
        topMuxOutput = -1;
        return;
    }

    int cumulativeMuxes = 0;
    for (topMux = 0; topMux < NUM_TOP_MUX; topMux++) {
        if (board * NUM_MUX_PER_BOARD + childMux < cumulativeMuxes + _muxPerTopMux[topMux]) {
            topMuxOutput = (board * NUM_MUX_PER_BOARD + childMux) - cumulativeMuxes;
            return;
        }
        cumulativeMuxes += _muxPerTopMux[topMux];
    }

    topMux = -1;
    topMuxOutput = -1;
}

void getBoardAndBoardKey(int key, int &board, int &boardKey) {
    if (key < 0) {
        board = -1;
        boardKey = -1;
        return;
    }

    board = key / NUM_KEYS_PER_BOARD;
    boardKey = key % NUM_KEYS_PER_BOARD;
}

// PEAK DETECT

void peakDetect(float voltage, int key) {
  int board, boardKey;
  // "static" variables keep their numbers between each run of this function
  static int state[NUM_KEYS];  // 0=idle, 1=looking for peak, 2=ignore aftershocks
  static float peak[NUM_KEYS];   // remember the highest reading
  static elapsedMillis msec[NUM_KEYS]; // timer to end states 1 and 2
  static bool playing[NUM_KEYS];

  if (_calibrated[key] == STATE_UNCALIB) {
      _zeroVoltage[key] = voltage;
      _calibrated[key] = STATE_CALIB;

      if (PLOT_OUTPUT == 0 ) {
        sprintf(_charVal, "CAL %d [%d] Zero=%f V", _calibrated[key], key, _zeroVoltage[key]);
        Serial.println(_charVal);
      }
  } 

  if (PLOT_OUTPUT == 2) {
    Serial.print("Key ");
    Serial.print(key);
    Serial.print(":");
    Serial.print(voltage);
    Serial.print(",");
  }

  float voltageSwing = fabs(_zeroVoltage[key] - voltage); 
  
  switch (state[key]) {
    // IDLE state: wait for any reading is above threshold.  Do not set
    // the threshold too low.  You don't want to be too sensitive to slight
    // vibration.
    case 0:
      if (voltageSwing > THRESHOLD_DELTA) {
        Serial.print("BEGIN [");
        Serial.print(key);
        Serial.print("] ");
        Serial.print(voltage);
        Serial.print(" OFF: ");
        Serial.println(_zeroVoltage[key]);

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
        int velocity = map(peak[key], THRESHOLD_DELTA, _maxSwing[key], 1, 127);
        if (PLOT_OUTPUT == 0) {      
          sprintf(_charVal, "PEAK [%d] %f %d", key, peak[key], velocity);
          Serial.println(_charVal);
        }
        if (peak[key] > _maxSwing[key]) {
          _maxSwing[key] = peak[key];
          velocity = map(peak[key], THRESHOLD_DELTA, _maxSwing[key], 1, 127);
          if (PLOT_OUTPUT == 0) {
            sprintf(_charVal, "CAL %d [%d] Off=%f On=%f", _calibrated[key], key, _zeroVoltage[key], _maxSwing[key]);
            Serial.println(_charVal);
          }
        }
        if (!playing[key]) {
          usbMIDI.sendNoteOn(_fields[board][boardKey].Note, velocity, _fields[board][boardKey].Channel);
          playing[key] = true;            
          _leds[key] = CRGB::White;
          FastLED.show();
        } else {
          usbMIDI.sendPolyPressure(_fields[board][boardKey].Note, velocity, _fields[board][boardKey].Channel); // Send aftertouch data
        }
        msec[key] = 0;
        state[key] = 2;
      }
      return;

    // Ignore Aftershock state: wait for things to be quiet azeroPoint.
    default:
      if (voltageSwing > THRESHOLD_DELTA) {
         msec[key] = 0; // keep resetting timer if above threshold
         state[key] = 1; 
      } else if (msec[key] > _aftershockMillis) {
        getBoardAndBoardKey(key, board, boardKey);
        usbMIDI.sendNoteOff(_fields[board][boardKey].Note, 0, _fields[board][boardKey].Channel);            
        _leds[key] = _fields[board][boardKey].Color;
        FastLED.show();
        playing[key] = false;
        state[key] = 0; // go back to idle when
      }
  }
}

void initializeCalibration() {
    for (int i = 0; i < NUM_KEYS; i++) {
        _calibrated[i] = 0;
    }
}

void setupMidi() {
  usbMIDI.setHandleSystemExclusive(midiHandleSystemEx);
  usbMIDI.setHandleProgramChange(midiHandleProgramChange);

  initializeCalibration();
}

void setupADC() {
  analogReadResolution(12);  // Set ADC resolution to 12 bits (0-4095)
  pinMode(A2, INPUT_DISABLE);
  pinMode(A16, INPUT_DISABLE);
  pinMode(A15, INPUT_DISABLE);
  pinMode(A17, INPUT_DISABLE);
  Serial.println("ADC intialized.");
}

// DAC
void setupDAC() {
  pinMode(MAINDAC_CS_PIN, OUTPUT);
  pinMode(OFFSETDAC_CS_PIN, OUTPUT);

  digitalWrite(MAINDAC_CS_PIN, HIGH);
  digitalWrite(OFFSETDAC_CS_PIN, HIGH);

  Serial.println("Setup DACs...");

  SPI.begin();
  SPI1.begin();
  delay(10);
  
  for (int key=0; key<NUM_KEYS; key++) {
    _gain[key] = 0;
    _offset[key] = 0;
  }

  Serial.println("DACs intialized.");
}


void mainDAC(uint16_t value, uint8_t channel)  //  channel = 0, 1
{
  uint16_t data = 0x3000 | value;
  if (channel == 1) data |= 0x8000;
  digitalWrite(MAINDAC_CS_PIN, LOW);
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
  SPI.transfer((uint8_t)(data >> 8));
  SPI.transfer((uint8_t)(data & 0xFF));
  SPI.endTransaction();
  digitalWrite(MAINDAC_CS_PIN, HIGH);
}

void offsetDAC(uint16_t value, uint8_t channel)  //  channel = 0, 1
{
  uint16_t data = 0x3000 | value;
  if (channel == 1) data |= 0x8000;
  digitalWrite(OFFSETDAC_CS_PIN, LOW);
  SPI1.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
  SPI1.transfer((uint8_t)(data >> 8));
  SPI1.transfer((uint8_t)(data & 0xFF));
  SPI1.endTransaction();
  digitalWrite(OFFSETDAC_CS_PIN, HIGH);
}


void performance_test()
{
  uint32_t start, stop;
  Serial.println();
  Serial.println(__FUNCTION__);

  start = micros();
  for (uint16_t value = 0; value < 4096; value++)
  {
    mainDAC(value, 0);
  }
  stop = micros();
  Serial.print("4096 calls:\t");
  Serial.print(stop - start);
  Serial.print("\t");
  Serial.println((stop - start) / 4096.0 );
  delay(10);
  
  start = micros();
  for (uint16_t value = 0; value < 4096; value++)
  {
    mainDAC(value, 1);
  }
  stop = micros();
  Serial.print("4096 calls:\t");
  Serial.print(stop - start);
  Serial.print("\t");
  Serial.println((stop - start) / 4096.0 );
  delay(10);
}

// CALIB
void loopCalibrationStart() {
  bool ok = loadCalibrationCSV();
  //ok = false;
  if (ok) { 
    analogReadAveraging(32);
    updateLEDs();

    _loopState = STATE_RUNNING;
    return;
  }
  _currentCalibrationKey = 0;
  _loopState = STATE_CALIBRATION_OFF;
}

void loopCalibrationOff() {
  setKey(_currentCalibrationKey);
  delay(10);

  analogReadAveraging(256);

  mainDAC(4095, 1); // set gain to 1
  mainDAC(0, 0); // set zeroPoint to center
  delayMicroseconds(30);

  int unscaledValue = analogRead(_adcPins[ADC_UNSCALED]);
  float unscaledVoltage = getAdcVoltage(unscaledValue);
  int dacValue = getDacValue(unscaledVoltage);
  mainDAC(dacValue, 0); // set zeroPoint to center
  delayMicroseconds(300);

  int scaledValue = analogRead(_adcPins[ADC_MAIN]);
  float scaledVoltage = getAdcVoltage(scaledValue);

  _println("[%d] Unscaled zero: %d %f V, scaled zero: %d %f V", _currentCalibrationKey, unscaledValue, unscaledVoltage, scaledValue, scaledVoltage);

  _zeroVoltage[_currentCalibrationKey] = scaledVoltage;
  _offset[_currentCalibrationKey] = dacValue;

  _leds[_currentCalibrationKey] = CRGB::Green;
  FastLED.show();

  _loopState = STATE_CALIBRATION_WAIT_ON;
}

void loopCalibrationWaitOn() {
  int currentValue = analogRead(_adcPins[ADC_MAIN]);
  float currentVoltage = getAdcVoltage(currentValue); 
  float swing = fabs(currentVoltage - _zeroVoltage[_currentCalibrationKey]);

  if (swing > THRESHOLD_DELTA) {
    _leds[_currentCalibrationKey] = CRGB::White;
    _maxSwing[_currentCalibrationKey] = swing;
    FastLED.show();

    _println("[%d] Over threshold: %f V Swing: %f V", _currentCalibrationKey, currentVoltage, swing);
    _loopState = STATE_CALIBRATION_ON;
  }
}

void loopCalibrationOn() {
  int currentValue = analogRead(_adcPins[ADC_MAIN]);
  float currentVoltage = getAdcVoltage(currentValue); 
  float swing = fabs(currentVoltage - _zeroVoltage[_currentCalibrationKey]);

  if (swing > _maxSwing[_currentCalibrationKey])
     _maxSwing[_currentCalibrationKey] = swing;

  _polarization[_currentCalibrationKey] = currentVoltage > _zeroVoltage[_currentCalibrationKey] ? 1 : -1;

  if (swing < THRESHOLD_DELTA) {
    float percentageFull = swing / MAX_VOLTAGE * 100;
    _println("[%d] Under threshold: %f V. Max swing: %f V (%f \%) Polarization: %d", 
        _currentCalibrationKey, 
        currentVoltage, 
        _maxSwing[_currentCalibrationKey], 
        percentageFull,
        _polarization[_currentCalibrationKey]);

    _leds[_currentCalibrationKey] = CRGB::Black;
    if (_currentCalibrationKey < NUM_KEYS - 1) {
      _currentCalibrationKey++;
      _loopState = STATE_CALIBRATION_OFF;
    } else
      _loopState = STATE_CALIBRATION_STOP;
  }
}

void loopCalibrationStop() {
  analogReadAveraging(32);
  saveCalibrationCSV();
  updateLEDs();

  _loopState = STATE_RUNNING;
}

void calibrationSweep(int& bestDacValue, float& closestError, int outputChannel, int measurePin, float targetVoltage) {
  int windowSize = 4095; // Initial window size is the full DAC range
  int steps = 16;        // Fixed number of steps in each cycle
  bestDacValue = 0;

  do { // Continue until the window is very small
    int startDacValue = max(0, bestDacValue - windowSize);
    int endDacValue = min(4095, bestDacValue + windowSize);
    closestError = 1e6; // Reset the closest error for this cycle
    
    for (int i = 0; i <= steps; i++) {
      int dacValue = startDacValue + i * (endDacValue - startDacValue) / steps;
      if (dacValue > DAC_RESOLUTION_MAX) break; // Avoid exceeding DAC limits

      mainDAC(dacValue, outputChannel);

      float dacVoltage = getDacVoltage(dacValue);
      int adcValue = analogRead(_adcPins[measurePin]);
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

float getAdcVoltage(int adcValue) {
  return (adcValue * ADC_VREF) / (float)ADC_RESOLUTION_MAX; 
}

float getDacVoltage(int dacValue) {
  return (dacValue * DAC_VREF) / (float)DAC_RESOLUTION_MAX; 
}
int getDacValue(float voltage) {
  return (voltage / DAC_VREF) * DAC_RESOLUTION_MAX;
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
        _offset[key] = line.substring(lastPos, nextPos).toFloat();
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
    }

    configFile.close();
    Serial.println("CSV calibration loaded.");
    return true;
}



void _println(const char* format, ...) {
  char buffer[128]; // Adjust buffer size as needed
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.println(buffer);
}
