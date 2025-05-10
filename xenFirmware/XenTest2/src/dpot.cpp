#include <Arduino.h>
#include <SPI.h>
#include "utils.h"
#include "dpot.h"
#include "xen.h"

void setupDPot() {
    pinMode(DPOT_CS_PIN, OUTPUT);
    digitalWrite(DPOT_CS_PIN, HIGH);

    SPI1.begin();
    dPot(0, 128);
    dPot(1, 128);
}

void dPot(byte address, byte value){
    digitalWrite(DPOT_CS_PIN, LOW); 
    SPI1.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    SPI1.transfer(address & 0x03);
    SPI1.transfer(value);
    SPI1.endTransaction();
    digitalWrite(DPOT_CS_PIN, HIGH);
}
  
void setDPot(int key, byte value) {  
    int channel = key < (NUM_KEYS_PER_BOARD * BOARDS_PER_DPOT_CHANNEL) ? 0 : 1;
    dPot(channel, value); 
}