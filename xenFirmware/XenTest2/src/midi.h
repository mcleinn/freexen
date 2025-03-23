#ifndef MIDI_H
#define MIDI_H

// CALIB
#define FIXED_GAIN 9.75

#define STATE_RUNNING 0
#define STATE_CALIBRATION_START 1
#define STATE_CALIBRATION_OFF 2
#define STATE_CALIBRATION_WAIT_ON 3
#define STATE_CALIBRATION_ON 4
#define STATE_CALIBRATION_STOP 5

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

extern CRGB* _leds;
extern byte _program;
extern XenField** _fields;

#endif // MIDI_H
