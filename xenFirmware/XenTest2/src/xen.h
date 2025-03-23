#ifndef XEN_H
#define XEN_H

#include <FastLED.h>

#define NUM_BOARDS 5
#define NUM_KEYS 280
#define NUM_LED_PER_KEY 2
#define NUM_KEYS_FOR_INPUT 112
#define NUM_KEYS_PER_BOARD 56

typedef struct {
    byte Board;
    byte BoardKey;
    byte Channel;
    byte Note;
    CRGB Color;     // Pin number such as GPIO_PIN_3, GPIO_PIN_5, etc.
} XenField;


#endif // XEN_H
