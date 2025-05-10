#ifndef ADC_H
#define ADC_H

#include <Arduino.h>

#define NUM_ADCS            4
#define ADC_MAIN1           0
#define ADC_UNSCALED1       1
#define ADC_MAIN2           2
#define ADC_UNSCALED2       3

const int BOARDS_PER_ADC_CHANNEL = 3;
const short _adcPins[NUM_ADCS] = { A1, A2, A4, A5 };
const float ADC_VREF = 3.3;       // Reference voltage in volts
const int ADC_RESOLUTION = 12;
const int ADC_RESOLUTION_MAX = 4095;  // 10 bit 1023, 12 bit 4095

void setupADC();
float getAdcVoltage(int channel);
int getAdcMain(int key);
int getAdcUnscaled(int key);

#endif // ADC_H
