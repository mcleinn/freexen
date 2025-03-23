#include <Arduino.h>
#include "utils.h"
#include "xen.h"
#include "adc.h"
#include "adc.h"

void setupADC() {
    analogReadResolution(12);  // Set ADC resolution to 12 bits (0-4095)
    for (int adc=0; adc<NUM_ADCS; adc++)
        pinMode(_adcPins[adc], INPUT_DISABLE);
    Serial.println("ADC intialized.");
}

float getAdcVoltage(int adcValue) {
    return (adcValue * ADC_VREF) / (float)ADC_RESOLUTION_MAX; 
}