#include <Arduino.h>
#include "utils.h"
#include "mux.h"
#include "xen.h"

int _ms_delay_after_mux = 10;

void setupMux() { 
    for (int i = 0; i < NUM_SELECTLINES; i++) {
      pinMode(_selectPins[0][i], OUTPUT);
    }
    for (int i = 0; i < NUM_SELECTLINES; i++) {
      pinMode(_selectPins[1][i], OUTPUT);
    }
    Serial.println("MUX initialized.");
    setMux(0, 0);
  }
  
  void setMux(int selectLine0, int selectLine1) {
    for (int i = 0; i < NUM_SELECTLINES; i++) {
      digitalWriteFast(_selectPins[0][i], (selectLine0 >> i) & 1);
    }
    for (int i = 0; i < NUM_SELECTLINES; i++) {
      digitalWriteFast(_selectPins[1][i], (selectLine1 >> i) & 1);
    }
  
    //Serial.print("MUX=");
    //Serial.print(selectLine0);
    //Serial.print("-");
    //Serial.print(selectLine1);
    //Serial.println("");
    delayMicroseconds(_ms_delay_after_mux);
  }
  
  
  void setKey(int key) {
    int board, childMux, childMuxOutput, topMux, topMuxOutput;
    getMuxAndOutput(key, board, childMux, childMuxOutput);
    getTopMuxAndOutput(board, childMux, topMux, topMuxOutput);
  
    setMux(childMuxOutput, topMuxOutput);
  }
  
  
  
  void getMuxAndOutput(int key, int &board, int &mux, int &output) {
      if (key < 0) {
          board = -1;
          mux = -1;
          output = -1;
          return;
      }
  
      int keysPerBoard = 0;
      for (int i = 0; i < NUM_MUX_PER_BOARD; i++) {
          keysPerBoard += _keysPerMux[i];
      }
  
      board = key / keysPerBoard;
      int localKey = key % keysPerBoard;
  
      int cumulativeKeys = 0;
      for (mux = 0; mux < NUM_MUX_PER_BOARD; mux++) {
          if (localKey < cumulativeKeys + _keysPerMux[mux]) {
              output = localKey - cumulativeKeys;
              return;
          }
          cumulativeKeys += _keysPerMux[mux];
      }
  
      board = -1;
      mux = -1;
      output = -1;
  }
  
  
  void getTopMuxAndOutput(int board, int childMux, int &topMux, int &topMuxOutput) {
      if (board < 0 || childMux < 0) {
          topMux = -1;
          topMuxOutput = -1;
          return;
      }
  
      int cumulativeMuxes = 0;
      for (topMux = 0; topMux < NUM_TOP_MUX; topMux++) {
          if (board * NUM_MUX_PER_BOARD + childMux < cumulativeMuxes + _muxPerTopMux[topMux]) {
              topMuxOutput = (board * NUM_MUX_PER_BOARD + childMux) - cumulativeMuxes;
              return;
          }
          cumulativeMuxes += _muxPerTopMux[topMux];
      }
  
      topMux = -1;
      topMuxOutput = -1;
  }
  
  void getBoardAndBoardKey(int key, int &board, int &boardKey) {
      if (key < 0) {
          board = -1;
          boardKey = -1;
          return;
      }
  
      board = key / NUM_KEYS_PER_BOARD;
      boardKey = key % NUM_KEYS_PER_BOARD;
  
      if (board >= NUM_BOARDS) {
         _println("Wrong key id: board to high %d > %d", board, NUM_BOARDS - 1);
        board = NUM_BOARDS - 1;
      }
      if (boardKey >= NUM_KEYS_PER_BOARD) {
        _println("Wrong key id: board key to high %d > %d", boardKey, NUM_KEYS_PER_BOARD - 1);
        boardKey = NUM_KEYS_PER_BOARD - 1;
      }
  }
  