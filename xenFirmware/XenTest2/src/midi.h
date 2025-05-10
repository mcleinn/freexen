#include <Adafruit_NeoPixel.h>

#ifndef MIDI_H
#define MIDI_H

// MIDI OUTPUT
#define BASE_NOTE         48


// SYSEX AND MTS
const int MTS_SYSEX_SIZE = 408;  // Expected size of the .syx file
const int MTS_NAME_OFFSET = 6;   // Byte index where the tuning name starts
const int MTS_NAME_LENGTH = 16;  // Length of the tuning name

void midiHandleSystemEx(const byte *data, uint16_t length, bool last);
void midiHandleProgramChange(byte channel, byte program);
void setupMidi();
void initializeCalibration();
bool mts_loadTuning();
void mts_sendTuning();
void mts_printTuningName();
void mts_receiveTuning(const byte *data, uint16_t length);
void sysEx_saveAndUpdate();
void sysEx_setField(const byte *data, uint16_t length);
void sysEx_noteOnOff(int command, const byte *data, uint16_t length);
void sysEx_update();

extern Adafruit_NeoPixel* LEDStrip;
extern byte _program;
extern XenField _fields[NUM_BOARDS][NUM_KEYS_PER_BOARD];

extern int _mainLoopState;
extern int _calibLoopState;
#endif // MIDI_H
