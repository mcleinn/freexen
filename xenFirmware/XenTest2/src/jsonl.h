#pragma once

// Minimal JSONL helpers shared across translation units.
// Keep this header Arduino-safe (no dynamic allocation).

// These symbols are defined in src/main.cpp.
extern int _outputFormat;

void beginJson(const char* type);
void endJson();
void printJsonKV(const char* k, const char* v, bool last=false);
void printJsonKV(const char* k, int v, bool last=false);
void printJsonKV(const char* k, float v, bool last=false);
