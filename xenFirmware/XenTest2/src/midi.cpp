#include <SD.h>
#include <Adafruit_NeoPixel.h>
#include "sdcard.h"
#include "midi.h"
#include "led.h"
#include "utils.h"
#include "xen.h"
#include "calib.h"

byte _currentTuning[MTS_SYSEX_SIZE];
bool _tuningLoaded = false;
extern int _manualCalibration;
extern bool _debugMode;

void setupMidi() {
    usbMIDI.setHandleSystemExclusive(midiHandleSystemEx);
    usbMIDI.setHandleProgramChange(midiHandleProgramChange);
}


// MIDI INCOMING
void midiHandleSystemEx(const byte *data, uint16_t length, bool last) {
  if (_debugMode) {
    Serial.print("SysEx Message: ");
    printBytes(data, length);
  }
  if (!last) {
    Serial.println(" (to be continued)");
    return;
  }

  Serial.println(" (end)");

  int command = (int8_t)(data[2]);
  if (command == 0) 
      sysEx_saveAndUpdate();
  else if (command == 1)
      sysEx_setField(data, length);
  else if (command == 2)
      mts_receiveTuning(data, length);
  else if (command == 3 || command == 4)
      sysEx_noteOnOff(command, data, length);
  else if (command == 5) { 
      for (int i=0; i < NUM_KEYS; i++) 
      LEDStrip->setPixelColor(i, 0, 0, 255);
      LEDStrip->show();
      _calibLoopState = STATE_CALIBRATION_START;
      _mainLoopState = 2;
      _manualCalibration = true;
  } else if (command == 6) {
      sysEx_update();
  }
}

void midiHandleProgramChange(byte channel, byte program) {
  _program = program;

  Serial.print("Program change: ");
  Serial.print(_program);
  Serial.println();

  loadConfigurationCSV();
  updateAllLEDs();

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
updateAllLEDs();
}

void sysEx_update() {
updateAllLEDs();
}


void sysEx_setField(const byte *data, uint16_t length) {
  int board = (byte)(data[3]);
  int boardKey = (byte)(data[4]);

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

  int channel = (byte)(data[5]);
  int note = (byte)(data[6]);

  Serial.print("SET [");
  Serial.print(board);
  
  Serial.print("-");
  Serial.print(boardKey);
  Serial.print("] ");
  
  _fields[board][boardKey].Board = board;
  _fields[board][boardKey].BoardKey = boardKey;
  _fields[board][boardKey].Channel = channel;
  _fields[board][boardKey].Note =  note;
  _fields[board][boardKey].r = mergeBytes(data[9], data[10]);
  _fields[board][boardKey].g = mergeBytes(data[11], data[12]);
  _fields[board][boardKey].b = mergeBytes(data[7], data[8]);

  int ledId = ledIdFromBoardKey(board, boardKey);
  updateLED(ledId);
}

void sysEx_noteOnOff(int command, const byte *data, uint16_t length) {
  int board = (byte)(data[3]);
  int boardKey = (byte)(data[4]);

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

  Serial.print("HIGHLIGHT [");
  Serial.print(board);
  
  Serial.print("-");
  Serial.print(boardKey);
  Serial.print("] ");
  
  if (command == 3) {
    u_int8_t r = mergeBytes(data[7], data[8]);
    u_int8_t g = mergeBytes(data[9], data[10]);
    u_int8_t b = mergeBytes(data[5], data[6]);

    int ledId = ledIdFromBoardKey(board, boardKey);
    LEDStrip->setPixelColor(ledId, r, g, b);
    LEDStrip->show();
  }
  if (command == 4) {
    updateAllLEDs();
  }
}