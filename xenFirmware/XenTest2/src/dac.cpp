#include <Arduino.h>
#include <SPI.h>
#include "utils.h"
#include "dac.h"
#include "xen.h"

int _us_delay_after_dac_zero = 30;
int _us_delay_after_dac_set = 300;

void setupDAC() {
    pinMode(MAINDAC_CS_PIN, OUTPUT);
    digitalWrite(MAINDAC_CS_PIN, HIGH);

    Serial.println("Setup DACs...");
    SPI.begin();
    delay(10);

    mainDAC(DAC_RESOLUTION_MAX / 2, 0);
    mainDAC(DAC_RESOLUTION_MAX / 2, 1);

    Serial.println("DACs intialized.");
}

void mainDAC(uint16_t value, uint8_t channel)  //  channel = 0, 1
{
    uint16_t data = 0x3000 | value;
    if (channel == 1) data |= 0x8000;
    digitalWrite(MAINDAC_CS_PIN, LOW);
    SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
    SPI.transfer((uint8_t)(data >> 8));
    SPI.transfer((uint8_t)(data & 0xFF));
    SPI.endTransaction();
    digitalWrite(MAINDAC_CS_PIN, HIGH);
}

void setDacMain(int key, uint16_t value) {  
    int channel = key < (NUM_KEYS_PER_BOARD * BOARDS_PER_DAC_CHANNEL) ? 0 : 1;
    mainDAC(value, channel); 
}

float getDacVoltage(int dacValue) {
    return (dacValue * DAC_VREF) / (float)DAC_RESOLUTION_MAX; 
}

int getDacValue(float voltage) {
    return (voltage / DAC_VREF) * DAC_RESOLUTION_MAX;
}