#ifndef DPOT_H
#define DPOT_H

#include <Arduino.h>

const int BOARDS_PER_DPOT_CHANNEL = 3;
#define DPOT_CS_PIN        0
#define DPOT_RESET_PIN    -1
#define DPOT_SHUTDOWN_PIN -1

void setupDPot();
void dPot(byte address, byte value);
void setDPot(int key, byte value);

#endif // DPOT_H
