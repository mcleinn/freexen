#ifndef LED_H
#define LED_H

#include "xen.h"

#define DATA_PIN 17

void setupLEDs(int numberLED);
void updateLEDs(int from, int to);
void updateAllLEDs();
void updateLED(int ledId);
int ledIdFromBoardKey(short b, short k);
void setColor(int key, int r, int g, int b);
void setColorAll(int r, int g, int b);

extern XenField _fields[NUM_BOARDS][NUM_KEYS_PER_BOARD];

#endif // LED_H
