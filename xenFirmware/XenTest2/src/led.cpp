#include <Adafruit_NeoPixel.h>
#include "led.h"
#include "xen.h"
#include "utils.h"

Adafruit_NeoPixel *LEDStrip = nullptr;    
int _numberLED = 0;

void setupLEDs(int numberLED) {
    if (numberLED == 0) numberLED = NUM_KEYS;
    if (numberLED >= NUM_KEYS) numberLED = NUM_KEYS;

    delete LEDStrip;                        // free any previous buffer
    LEDStrip = new Adafruit_NeoPixel(numberLED, DATA_PIN, NEO_GRB + NEO_KHZ800);
  
    LEDStrip->begin();
    LEDStrip->setBrightness(255);
    
    _numberLED = numberLED;
  
    for (int i=0; i < _numberLED; i++)
      LEDStrip->setPixelColor(i, 0, 0, 255);
    LEDStrip->show();
}

void updateLED(int ledId) {
  if (_numberLED == 0) return;
  int board, boardKey;
  getBoardAndBoardKey(ledId, board, boardKey);

  LEDStrip->setPixelColor(ledId,  
    LEDStrip->gamma8(_fields[board][boardKey].r), 
    LEDStrip->gamma8(_fields[board][boardKey].g), 
    LEDStrip->gamma8(_fields[board][boardKey].b));
}

void updateAllLEDs() {
  updateLEDs(0, _numberLED-1);
}

void updateLEDs(int from, int to) {
  if (_numberLED == 0) return;
  // Turn the LED on, then pause
  for (int ledId = from; ledId <= to; ledId++) {
     if (ledId < 0 || ledId >= _numberLED) break;
     updateLED(ledId);
  }
  LEDStrip->show();
}

// board, key, led number -> unique ledId
int ledIdFromBoardKey(short b, short k) {
  if (b >= NUM_BOARDS) {
    _println("Wrong LED id: board %d, max %d", b, NUM_BOARDS-1);
    return NUM_BOARDS - 1;
  }
  if (k >= NUM_KEYS_PER_BOARD ) {
    _println("Wrong LED id: keys per board %d, max %d", k, NUM_KEYS_PER_BOARD-1);
    return NUM_KEYS_PER_BOARD - 1;
  }
  return b * NUM_KEYS_PER_BOARD + k;
}

void setColor(int key, int r, int g, int b) {
  if (_numberLED == 0) return;
  
  int board, boardKey;
  getBoardAndBoardKey(key, board, boardKey);
  _fields[board][boardKey].r = r;
  _fields[board][boardKey].g = g;
  _fields[board][boardKey].b = b;
}

void setColorAll(int r, int g, int b) {
  if (_numberLED == 0) return;

  int board, boardKey;
  for (int key=0; key < _numberLED; key++) {
    getBoardAndBoardKey(key, board, boardKey);
    _fields[board][boardKey].r = r;
    _fields[board][boardKey].g = g;
    _fields[board][boardKey].b = b;
  }
}
