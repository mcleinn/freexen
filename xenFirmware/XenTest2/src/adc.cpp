#include <Arduino.h>
#include "utils.h"
#include "xen.h"
#include "adc.h"

void setupADC() {
    analogReadResolution(ADC_RESOLUTION);  // Set ADC resolution to 12 bits (0-4095)
    analogReadAveraging(_measureAvgStandard);
    for (int adc=0; adc<NUM_ADCS; adc++)
        pinMode(_adcPins[adc], INPUT_DISABLE);
    _println("ADC intialized, averaging %d.", _measureAvgStandard);
}

int getAdcUnscaled(int key) {
    const short adc = key < (NUM_KEYS_PER_BOARD  * BOARDS_PER_ADC_CHANNEL) ? ADC_UNSCALED1 : ADC_UNSCALED2;
    return analogRead(_adcPins[adc]);
}

int getAdcUnscaledRawByIndex(int adcIndex)
{
    if (adcIndex < 0 || adcIndex >= NUM_ADCS) return 0;
    return analogRead(_adcPins[adcIndex]);
}

float getAdcVoltage(int adcValue) {
    return (adcValue * ADC_VREF) / (float)ADC_RESOLUTION_MAX; 
}
