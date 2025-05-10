#ifndef SDCARD_H
#define SDCARD_H

#include <Arduino.h>
#include <SD.h>
#include "xen.h"

void setupCard();
void loadConfigurationCSV();
void saveConfigurationCSV();

extern byte _program;
extern XenField _fields[NUM_BOARDS][NUM_KEYS_PER_BOARD];

#endif // SDCARD_H
