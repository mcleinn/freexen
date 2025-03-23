#ifndef ADC_H
#define ADC_H

#include <Arduino.h>

#define NUM_ADCS            4
#define ADC_MAIN1           0
#define ADC_UNSCALED1       1
#define ADC_MAIN2           3
#define ADC_UNSCALED2       4

const short _adcPins[NUM_ADCS] = { A1, A2, A4, A5 };
const float ADC_VREF = 3.3;       // Reference voltage in volts
const int ADC_RESOLUTION = 12;
const int ADC_RESOLUTION_MAX = 4095;  // 10 bit 1023, 12 bit 4095

void setupADC();
float getAdcVoltage(int channel);

#endif // ADC_H
