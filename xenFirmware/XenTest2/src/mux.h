#ifndef MUX_H
#define MUX_H

#include <Arduino.h>

#define NUM_TOP_MUX        2
#define NUM_MUX_PER_BOARD  4
#define NUM_SELECTLINES    4

const int _selectPins[NUM_TOP_MUX][NUM_SELECTLINES] = { {2, 3, 4, 5}, {6, 7, 8, 9} }; // Change these pin numbers based on your actual wiring
const int _keysPerMux[NUM_MUX_PER_BOARD] = { 15, 15, 15, 11 };
const int _muxPerTopMux[NUM_TOP_MUX] = { 12, 12 };
#define INPUT_BUFFER_SIZE 100


void setupMux();
void setMux(int board, int childMux);
void getMuxAndOutput(int key, int &board, int &childMux, int &childMuxOutput);
void getTopMuxAndOutput(int board, int childMux, int &topMux, int &topMuxOutput);
void setKey(int key);

#endif // MUX_H
