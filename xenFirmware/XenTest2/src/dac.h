#ifndef DAC_H
#define DAC_H

#include <Arduino.h>

#define MAINDAC_CS_PIN    10

const int BOARDS_PER_DAC_CHANNEL = 3;
const int DAC_RESOLUTION = 12;
const int DAC_RESOLUTION_MAX = 4095;  // 10 bit 1023, 12 bit 4095
const int DAC_VREF = 5;

void setupDAC();
void mainDAC(uint16_t value, uint8_t channel);
float getDacVoltage(int dacValue);
int getDacValue(float voltage);
void setDacMain(int key, uint16_t value);

#endif // DAC_H
