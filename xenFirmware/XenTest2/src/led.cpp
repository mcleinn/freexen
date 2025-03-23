#include<FastLED.h>
#include "led.h"
#include "xen.h"
#include "utils.h"

CRGB _leds[NUM_KEYS * NUM_LED_PER_KEY];

void setupLEDs() {
    FastLED.addLeds<WS2812B, DATA_PIN, GRB>(_leds, NUM_KEYS * NUM_LED_PER_KEY);  
  
    // Turn the LED on, then pause
    for (int i=0; i < NUM_KEYS * NUM_LED_PER_KEY; i++)
      _leds[i] = CRGB::Blue;
    FastLED.show();
}


void updateLEDs() {
    Serial.println("Updating LEDs...");
    // Turn the LED on, then pause
    for (int b = 0; b < NUM_BOARDS; b++) {
        for (int c = 0; c < NUM_KEYS_PER_BOARD; c++) {    
        //_println("UPD [%d] %d %d %d", l, _fields[b][c].Color.r, _fields[b][c].Color.g, _fields[b][c].Color.b);
        for (int l = 0; l < NUM_LED_PER_KEY; l++) {
            int ledId = ledIdFromBoardKey(b, c, l);
            _leds[ledId] = _fields[b][c].Color;
        }
        }
    }
    FastLED.show();
    Serial.println("LEDs updated.");
}

// board, key, led number -> unique ledId
int ledIdFromBoardKey(short b, short k, short l) {
    if (b >= NUM_BOARDS) {
      _println("Wrong LED id: board %d, max %d", b, NUM_BOARDS-1);
      return NUM_BOARDS - 1;
    }
    if (k >= NUM_KEYS_PER_BOARD ) {
      _println("Wrong LED id: keys per board %d, max %d", k, NUM_KEYS_PER_BOARD-1);
      return NUM_KEYS_PER_BOARD - 1;
    }
    if (l >= NUM_LED_PER_KEY) {
      _println("Wrong LED id: led per key %d, max %d", l, NUM_LED_PER_KEY-1);
      return NUM_LED_PER_KEY - 1;
    }
    return b * (NUM_KEYS_PER_BOARD * NUM_LED_PER_KEY) + k + l * NUM_KEYS_PER_BOARD;
  }
  