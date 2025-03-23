#include <Arduino.h>
#include <SPI.h>
#include "MCP_DAC.h"

#include "main.h"
#include "sdcard.h"
#include "adc.h"
#include "dpot.h"
#include "dac.h"
#include "mux.h"
#include "xen.h"
#include "led.h"
#include "utils.h"
#include "midi.h"

int _loop = 0;
int _channel = 0;

void setup() {
  Serial.begin(115200);
  
  setupCard();
  setupMux();
  setupADC();
  setupDAC();
  setupDPot();
  setupLEDs();
  setupMidi();

  Serial.println("Ready to receive commands (e.g. a123)...");
}

void loop() {
  static char inputBuffer[INPUT_BUFFER_SIZE];
  static int inputIndex = 0;

  while (Serial.available() > 0) {
    char incomingByte = (char)Serial.read();
    
    if (incomingByte == '\n') {
      // Terminate the string
      inputBuffer[inputIndex] = '\0';
      // Process the command
      parseCommand(inputBuffer);
      // Reset the buffer index for the next command
      inputIndex = 0;
    } else {
      // Only add character if there is room in the buffer
      if (inputIndex < INPUT_BUFFER_SIZE - 1) {
        inputBuffer[inputIndex++] = incomingByte;
      }
      // Optionally, handle buffer overflow if needed
    }
  }

  if (_loop == 1) {
    // put your main code here, to run repeatedly:
    int adcMain1 = analogRead(_adcPins[ADC_MAIN1]);
    int adcUnscaled1 = analogRead(_adcPins[ADC_UNSCALED1]);

    float scaledVoltage1 = getAdcVoltage(adcMain1);
    float unscaledVoltage1 = getAdcVoltage(adcUnscaled1);

    _println("Min:0.0,Scaled:%f,Unscaled:%f,Max:3.3", scaledVoltage1, unscaledVoltage1);
    delay(100);
  }
}


void parseCommand(char* inputBuffer) {
  // Sanity check: Must have at least one letter and one digit
  if (strlen(inputBuffer) < 2) {
    return; // Not a valid command
  }

  // The first character should be our letter command
  char cmd = inputBuffer[0];

  // Everything after the first character should be digits
  // Use toInt() to convert the substring into an integer
  int value = atoi(inputBuffer + 1);

  // Now do something with cmd and value
  Serial.print("Command: ");
  Serial.print(cmd);
  Serial.print("  Value: ");
  Serial.println(value);

  // Add your own logic for each command here:
  switch(cmd) {
     case 'l':
       _loop = value;
       break;
     case 'k':
       setKey(value);
     case 'c':
       _channel = value;
     case 'o':
       mainDAC(value, _channel);
     case 'g':
       dPot(_channel, value);
     default:
       // Unknown command
       break;
  }
}


