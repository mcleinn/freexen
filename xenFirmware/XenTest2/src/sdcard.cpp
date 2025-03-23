#include "sdcard.h"
#include "xen.h"

const int _sdCardChipSelect = BUILTIN_SDCARD;

void setupCard() {
    Serial.println("Initializing SD card...");
 
   if (!SD.begin(_sdCardChipSelect)) {
     Serial.println("SD card init failed!");
     return;
   }
   Serial.println("SD card initialized.");
 }
 

 void saveConfigurationCSV() {
    char filename[255];
    sprintf(filename, "config%d.csv", _program);
    Serial.println(filename);
    SD.remove(filename);
    File configFile = SD.open(filename, FILE_WRITE);
    if (configFile) {
        Serial.println("Saving configuration to CSV...");
        for (int i = 0; i < NUM_BOARDS; i++) {
            for (int j = 0; j < NUM_KEYS_PER_BOARD; j++) {
                XenField field = _fields[i][j];
                configFile.print(field.Board);
                configFile.print(",");
                configFile.print(field.BoardKey);
                configFile.print(",");
                configFile.print(field.Channel);
                configFile.print(",");
                configFile.print(field.Note);
                configFile.print(",");
                configFile.print(field.Color.r);
                configFile.print(",");
                configFile.print(field.Color.g);
                configFile.print(",");
                configFile.print(field.Color.b);
                configFile.println();
            }
        }
        configFile.close();
        Serial.println("CSV configuration saved.");
    } else {
        Serial.println("Error opening config file for writing.");
    }
}
 
void loadConfigurationCSV() {
    char filename[255];
    sprintf(filename, "config%d.csv", _program);
    Serial.println(filename);
    File configFile = SD.open(filename, FILE_READ);
    if (configFile) {
        Serial.println("Loading configuration from CSV...");
        int boardIndex = 0;
        while (configFile.available() && boardIndex < NUM_BOARDS) {
            String line = configFile.readStringUntil('\n');
            int lastPos = 0, nextPos = 0;
            XenField field;

            nextPos = line.indexOf(',', lastPos);
            field.Board = (byte)line.substring(lastPos, nextPos).toInt();
            lastPos = nextPos + 1;

            nextPos = line.indexOf(',', lastPos);
            field.BoardKey = (byte)line.substring(lastPos, nextPos).toInt();
            lastPos = nextPos + 1;

            nextPos = line.indexOf(',', lastPos);
            field.Channel = (byte)line.substring(lastPos, nextPos).toInt();
            lastPos = nextPos + 1;

            nextPos = line.indexOf(',', lastPos);
            field.Note = (byte)line.substring(lastPos, nextPos).toInt();
            lastPos = nextPos + 1;

            nextPos = line.indexOf(',', lastPos);
            field.Color.r = (byte)line.substring(lastPos, nextPos).toInt();
            lastPos = nextPos + 1;

            nextPos = line.indexOf(',', lastPos);
            field.Color.g = (byte)line.substring(lastPos, nextPos).toInt();
            lastPos = nextPos + 1;

            nextPos = line.indexOf(',', lastPos);
            field.Color.b = (byte)line.substring(lastPos, nextPos).toInt();
            
            if (field.Board >= 0 && field.Board < NUM_BOARDS && field.Board >= 0 && field.BoardKey < NUM_KEYS_PER_BOARD) {
              _fields[field.Board][field.BoardKey] = field;
              //_println("[%d] %d %d %d", field.Board * 56 + field.BoardKey, field.Color.r, field.Color.g, field.Color.b);
            }
        }
        configFile.close();
        Serial.println("CSV configuration loaded.");
    } else {
        Serial.println("Error opening config file for reading.");
    }
}