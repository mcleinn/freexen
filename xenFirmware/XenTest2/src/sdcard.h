#ifndef SDCARD_H
#define SDCARD_H

#include <Arduino.h>
#include <SD.h>
#include "xen.h"

void setupCard();
void loadConfigurationCSV();
void saveConfigurationCSV();

extern byte _program;
extern XenField** _fields;

#endif // SDCARD_H
