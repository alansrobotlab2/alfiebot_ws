#include <Arduino.h>
#include <Preferences.h>


#define BOARDCONFIG 1


Preferences boardconfig;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(4000);
  
  Serial.println("ESP32 EEPROM Programming - Board Configuration");
  
  // Open Preferences with namespace "boardconfig" in read/write mode
  boardconfig.begin("boardconfig", false);
  
  // Store the boardid with value 
  boardconfig.putUInt("boardid", BOARDCONFIG);
  Serial.println("Board ID written to EEPROM:" + String(BOARDCONFIG));
  
  // Read back the value to verify
  unsigned int readValue = boardconfig.getUInt("boardid", 999);
  Serial.print("Board ID read from EEPROM: ");
  Serial.println(readValue);
  
  // Close the Preferences
  boardconfig.end();
  
  Serial.println("Programming complete!");
}

void loop() {
  // Nothing to do in loop
}