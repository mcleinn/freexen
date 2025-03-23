#ifndef LED_H
#define LED_H

#include "xen.h"

#define DATA_PIN 17
void setupLEDs();
void updateLEDs();
int ledIdFromBoardKey(short b, short k, short l);

extern XenField** _fields;

#endif // LED_H
