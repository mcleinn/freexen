#ifndef DPOT_H
#define DPOT_H

#include <Arduino.h>

#define DPOT_CS_PIN        0
#define DPOT_RESET_PIN    -1
#define DPOT_SHUTDOWN_PIN -1

void setupDPot();
void dPot(byte address, byte value);

#endif // DPOT_H
