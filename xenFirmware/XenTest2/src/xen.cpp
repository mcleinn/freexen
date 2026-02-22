#include <Arduino.h>
#include "utils.h"
#include "xen.h"

int _fromKey = 0, _toKey = NUM_KEYS;
XenField _fields[NUM_BOARDS][NUM_KEYS_PER_BOARD];
byte _program = 7;


void setKeyInterval(int from, int to) {   
    _fromKey = from;
    _toKey = to;
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
       _println("Wrong key id: board too high %d > %d", board, NUM_BOARDS - 1);
      board = NUM_BOARDS - 1;
    }
    if (boardKey >= NUM_KEYS_PER_BOARD) {
      _println("Wrong key id: board key too high %d > %d", boardKey, NUM_KEYS_PER_BOARD - 1);
      boardKey = NUM_KEYS_PER_BOARD - 1;
    }
}