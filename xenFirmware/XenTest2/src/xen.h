#include<Arduino.h>

#ifndef XEN_H
#define XEN_H

#define NUM_BOARDS 5
#define NUM_KEYS 280
#define NUM_KEYS_PER_BOARD 56

typedef struct {
    byte Board;
    byte BoardKey;
    byte Channel;
    byte Note;
    u_int8_t r;  
    u_int8_t g;
    u_int8_t b;  
} XenField;

void setKeyInterval(int from, int to);
void getBoardAndBoardKey(int key, int &board, int &boardKey);

#endif // XEN_H
