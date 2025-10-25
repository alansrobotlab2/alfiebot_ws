/**
 * RP2040 I2C Slave Device
 * 
 * This program configures the RP2040 as an I2C slave on address 0x58
 * 
 * See config.h for pin assignments and configuration details
 */

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "DriverBoard.h"
#include "ws2812/ws2812.h"

// Heartbeat pattern: double pulse, then pause
const bool heartbeatPattern[] = {1, 0, 1, 0, 0, 0, 0, 0};
const uint8_t heartbeatPatternLength = sizeof(heartbeatPattern) / sizeof(heartbeatPattern[0]);

// Create WS2812 LED object
WS2812 led(WS2812_PIN);

// Create DriverBoard instance to hold device state
DriverBoard rp;

// I2C Watchdog tracking
volatile unsigned long lastI2CActivity = 0;
volatile bool i2cActive = false;

// Data buffers
volatile uint8_t receiveBuffer[32];
volatile uint8_t receiveIndex = 0;
volatile uint8_t sendBuffer[32];
volatile uint8_t sendLength = 0;

// ============================================================================
// Debug Output Helper
// ============================================================================

/**
 * Check if serial is available and print (non-blocking)
 * Only prints if Serial is connected to avoid delays
 */
template<typename T>
void debugPrint(T msg) {
  if (Serial) Serial.print(msg);
}

template<typename T>
void debugPrintln(T msg) {
  if (Serial) Serial.println(msg);
}

void debugPrintln() {
  if (Serial) Serial.println();
}

// ============================================================================
// Sensor Reading Functions (Placeholders)
// ============================================================================

/**
 * Read the back switch state
 * @return true if pressed, false if not pressed
 */
bool readBackSwitch() {
  // Read with pullup - HIGH when at lower limit
  return digitalRead(BACK_SWITCH_PIN) == HIGH;
}

/**
 * Read the chip temperature
 * @return Temperature in whole degrees Celsius
 */
uint8_t readChipTemperature() {
  // RP2040 has internal temperature sensor on ADC channel 4
  // Formula from RP2040 datasheet:
  // T = 27 - (ADC_voltage - 0.706) / 0.001721
  
  // Read ADC value (12-bit: 0-4095)
  // Note: Arduino-Mbed uses analogRead() which returns 0-1023 by default
  // We need to read the actual voltage
  float adcVoltage = analogRead(TEMP_ADC_PIN) * (3.3 / 1023.0);
  
  // Calculate temperature using RP2040 formula
  float tempC = 27.0 - ((adcVoltage - 0.706) / 0.001721);
  
  // Convert to uint8_t whole degrees
  uint8_t temp = (uint8_t)(tempC + 0.5);  // Round to nearest degree
  
  return temp;
}

// ============================================================================
// PWM Control Functions
// ============================================================================

// Helper function to set left eye PWM
void setLeftEyePWM(uint8_t pwmValue) {
  rp.leftEyePWM = pwmValue;
  analogWrite(PWMA_PIN, pwmValue);
  if (Serial) {
    Serial.print("Left Eye PWM set to: ");
    Serial.println(pwmValue);
  }
}

// Helper function to set right eye PWM
void setRightEyePWM(uint8_t pwmValue) {
  rp.rightEyePWM = pwmValue;
  analogWrite(PWMB_PIN, pwmValue);
  if (Serial) {
    Serial.print("Right Eye PWM set to: ");
    Serial.println(pwmValue);
  }
}

// ============================================================================
// I2C Command Processing
// ============================================================================

/**
 * Build status packet for I2C response
 */
void buildStatusPacket() {
  sendBuffer[0] = rp.leftEyePWM;
  sendBuffer[1] = rp.rightEyePWM;
  sendBuffer[2] = rp.backSwitchPressed ? 1 : 0;
  sendBuffer[3] = rp.chipTemperature;  // Whole degrees C
  sendLength = 4;  // Now 4 bytes instead of 5
}

/**
 * Process CMD_SET_EYES command
 * @param buffer Command buffer
 * @param length Buffer length
 */
void processSetEyesCommand(uint8_t* buffer, uint8_t length) {
  if (length < 3) {
    if (Serial) Serial.println("ERROR: SET_EYES requires 3 bytes [CMD, LEFT_PWM, RIGHT_PWM]");
    return;
  }
  
  uint8_t leftPWM = buffer[1];
  uint8_t rightPWM = buffer[2];
  
  setLeftEyePWM(leftPWM);
  setRightEyePWM(rightPWM);
  
  if (Serial) {
    Serial.print("Eyes set - Left: ");
    Serial.print(leftPWM);
    Serial.print(", Right: ");
    Serial.println(rightPWM);
  }
}

/**
 * Process CMD_GET_STATUS command
 * Prepares status packet for next I2C read operation
 */
void processGetStatusCommand() {
  // Update sensor readings
  rp.backSwitchPressed = readBackSwitch();
  rp.chipTemperature = readChipTemperature();
  
  // Build status packet
  buildStatusPacket();
  
  if (Serial) {
    Serial.println("Status requested - packet prepared");
    Serial.print("  Left PWM: ");
    Serial.println(rp.leftEyePWM);
    Serial.print("  Right PWM: ");
    Serial.println(rp.rightEyePWM);
    Serial.print("  Back Switch: ");
    Serial.println(rp.backSwitchPressed ? "PRESSED" : "NOT PRESSED");
    Serial.print("  Temperature: ");
    Serial.print(rp.chipTemperature);
    Serial.println("°C");
  }
}

// Process I2C commands
void processI2CCommand(uint8_t* buffer, uint8_t length) {
  if (length < 1) return;  // Need at least a command byte
  
  uint8_t command = buffer[0];
  
  switch (command) {
    case CMD_SET_EYES:
      processSetEyesCommand(buffer, length);
      break;
      
    case CMD_GET_STATUS:
      processGetStatusCommand();
      break;
      
    default:
      if (Serial) {
        Serial.print("Unknown command: 0x");
        Serial.println(command, HEX);
      }
      break;
  }
}

// I2C Receive Handler - called when master sends data to this slave
void receiveEvent(int numBytes) {
  receiveIndex = 0;
  
  // Read all bytes from the I2C master
  while (Wire.available() && receiveIndex < sizeof(receiveBuffer)) {
    receiveBuffer[receiveIndex++] = Wire.read();
  }
  
  // Update I2C watchdog
  lastI2CActivity = millis();
  i2cActive = true;
  
  // Echo received data for debugging (only if Serial connected)
  if (Serial) {
    Serial.print("I2C Received ");
    Serial.print(receiveIndex);
    Serial.print(" bytes: ");
    for (int i = 0; i < receiveIndex; i++) {
      Serial.print("0x");
      Serial.print(receiveBuffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// I2C Request Handler - called when master requests data from this slave
void requestEvent() {
  // Update I2C watchdog on read operations too
  lastI2CActivity = millis();
  i2cActive = true;
  
  // Send the buffered data back to master
  if (sendLength > 0) {
    Wire.write((uint8_t*)sendBuffer, sendLength);
    if (Serial) {
      Serial.print("I2C Sent ");
      Serial.print(sendLength);
      Serial.println(" bytes");
    }
  } else {
    // Send a default response if no data is ready
    uint8_t response = 0xAA;
    Wire.write(response);
    if (Serial) Serial.println("I2C Sent default byte: 0xAA");
  }
}

void setup() {
  // Initialize WS2812B RGB LED
  if (!led.begin()) {
    // If LED initialization fails, continue anyway (no Serial yet)
  }

  // Initialize serial communication for debugging (non-blocking)
  Serial.begin(115200);
  // Don't wait for serial connection - continue immediately
  
  Serial.println("RP2040 I2C Slave Starting...");
  
  // Initialize Motor/LED PWM pins
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  
  // Initialize back switch pin with internal pullup
  pinMode(BACK_SWITCH_PIN, INPUT_PULLUP);
  
  // Note: Temperature sensor (ADC4) is internal, no pin initialization needed
  
  // Set direction pins (AIN1=HIGH, AIN2=LOW for left; BIN1=LOW, BIN2=HIGH for right)
  digitalWrite(AIN1_PIN, HIGH);
  digitalWrite(AIN2_PIN, LOW);
  digitalWrite(BIN1_PIN, LOW);
  digitalWrite(BIN2_PIN, HIGH);
  
  // Configure PWM - Arduino-Mbed uses default 500Hz, we'll set it per-pin
  // Note: Arduino-Mbed doesn't support analogWriteFrequency, so we use default PWM
  // The hardware will run at the framework's configured frequency
  analogWriteResolution(PWM_RESOLUTION);
  
  // Set initial PWM to 0
  setLeftEyePWM(0);
  setRightEyePWM(0);
  
  Serial.println("Motor PWM initialized:");
  Serial.print("  Left Eye: PWMA=GPIO");
  Serial.print(PWMA_PIN);
  Serial.print(", AIN1=GPIO");
  Serial.print(AIN1_PIN);
  Serial.print(", AIN2=GPIO");
  Serial.println(AIN2_PIN);
  Serial.print("  Right Eye: PWMB=GPIO");
  Serial.print(PWMB_PIN);
  Serial.print(", BIN1=GPIO");
  Serial.print(BIN1_PIN);
  Serial.print(", BIN2=GPIO");
  Serial.println(BIN2_PIN);
  Serial.println("  PWM: 8-bit resolution (0-255)");
  Serial.print("  Back Switch: GPIO");
  Serial.println(BACK_SWITCH_PIN);
  Serial.print("  Temperature Sensor: ADC");
  Serial.println(TEMP_ADC_PIN);
  
  // Initialize I2C as slave on address 0x58
  // Using default pins: SDA=GPIO4, SCL=GPIO5
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.setClock(I2C_CLOCK_SPEED);  // Set Fast Mode: 400 kHz
  
  // Register I2C event handlers
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  Serial.print("I2C Slave initialized on address 0x");
  Serial.println(I2C_SLAVE_ADDRESS, HEX);
  Serial.print("I2C Clock Speed: ");
  Serial.print(I2C_CLOCK_SPEED / 1000);
  Serial.println(" kHz (Fast Mode)");
  Serial.println("SDA: GPIO 4, SCL: GPIO 5");
  Serial.println("WS2812B LED on GPIO 16");
  Serial.println();
  Serial.println("I2C Protocol:");
  Serial.println("  CMD 0x01: Set Eyes [0x01, LEFT_PWM, RIGHT_PWM]");
  Serial.println("  CMD 0x02: Get Status (returns 5-byte status packet)");
  Serial.println();
  Serial.println("Ready to receive commands...");
  
  // Startup animation - rainbow cycle
  led.rainbowCycle(10);
  
  // Initialize I2C watchdog
  lastI2CActivity = millis();
  i2cActive = false;
  
  // Start with LED off
  led.clear();
}

void loop() {
  static unsigned long lastHeartbeatUpdate = 0;
  static uint8_t heartbeatIndex = 0;
  static bool watchdogWarningShown = false;
  
  unsigned long currentTime = millis();
  
  // Process received I2C data
  if (receiveIndex > 0) {
    // Process the command
    processI2CCommand((uint8_t*)receiveBuffer, receiveIndex);
    
    receiveIndex = 0;  // Clear receive buffer
  }
  
  // Check I2C watchdog - has there been activity in the last 100ms?
  bool i2cHealthy = (currentTime - lastI2CActivity) < I2C_WATCHDOG_TIMEOUT_MS;
  
  // Update watchdog warning state
  if (!i2cHealthy && i2cActive && !watchdogWarningShown) {
    if (Serial) Serial.println("WARNING: I2C communication timeout!");
    watchdogWarningShown = true;
  } else if (i2cHealthy && watchdogWarningShown) {
    if (Serial) Serial.println("I2C communication restored");
    watchdogWarningShown = false;
  }
  
  // Update LED pattern
  if (i2cHealthy && i2cActive) {
    // Green heartbeat pattern when I2C is healthy
    if (currentTime - lastHeartbeatUpdate >= HEARTBEAT_PULSE_MS) {
      lastHeartbeatUpdate = currentTime;
      
      if (heartbeatPattern[heartbeatIndex]) {
        led.setColor(0, 255, 0);  // Green pulse
      } else {
        led.clear();  // Off
      }
      
      // Advance to next heartbeat pattern index
      heartbeatIndex = (heartbeatIndex + 1) % heartbeatPatternLength;
    }
  } else {
    // Red blinking at 4 Hz when I2C timeout or no communication
    static bool blinkState = false;
    if (currentTime - lastHeartbeatUpdate >= TIMEOUT_BLINK_MS) {
      lastHeartbeatUpdate = currentTime;
      blinkState = !blinkState;
      
      if (blinkState) {
        led.setColor(255, 0, 0);  // Red on
      } else {
        led.clear();  // Off
      }
    }
  }
  
  // Small delay to prevent tight looping
  delay(1);
}
