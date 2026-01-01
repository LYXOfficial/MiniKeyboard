#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPR121.h>
#include "BLEKeyboard.h"

#define SDA_PIN 2
#define SCL_PIN 1
#define MPR121_ADDR1 0x5A
#define MPR121_ADDR2 0x5B

// SoftUSB pins - D+ on GPIO19, D- on GPIO18
#define USB_DP_PIN 19
#define USB_DM_PIN 18

Adafruit_MPR121 mpr1 = Adafruit_MPR121();
Adafruit_MPR121 mpr2 = Adafruit_MPR121();

// Using BLE keyboard instead of SoftUSB

// baseline thresholds per electrode
const uint8_t btnbaseline[24] = {11, 11, 12, 11, 10, 11, 12, 9,
                                 10, 11, 11, 10, 9, 8,  7,  6,
                                 9, 13, 13, 13, 13, 10, 10, 10};

// HID keycode mapping for 24 electrodes. 0 means no mapping.
// Using USB HID keycodes (see USB HID Usage Tables)
#define KEY_NUM_LOCK        0x53
#define KEY_KP_DIVIDE       0x54
#define KEY_KP_MULTIPLY     0x55
#define KEY_KP_SUBTRACT     0x56
#define KEY_KP_ADD          0x57
#define KEY_KP_ENTER        0x58
#define KEY_KP_1            0x59
#define KEY_KP_2            0x5A
#define KEY_KP_3            0x5B
#define KEY_KP_4            0x5C
#define KEY_KP_5            0x5D
#define KEY_KP_6            0x5E
#define KEY_KP_7            0x5F
#define KEY_KP_8            0x60
#define KEY_KP_9            0x61
#define KEY_KP_0            0x62
#define KEY_KP_DECIMAL      0x63

// Macro keys (sensors 20-23) mapped to special functions
#define KEY_F13             0x68
#define KEY_F14             0x69
#define KEY_F15             0x6A
#define KEY_F16             0x6B

const uint8_t hidmap[24] = {
    KEY_NUM_LOCK,      // 0  NumLock
    KEY_KP_DIVIDE,     // 1  /
    KEY_KP_MULTIPLY,   // 2  *
    KEY_KP_SUBTRACT,   // 3  -
    KEY_KP_7,          // 4  7
    KEY_KP_8,          // 5  8
    KEY_KP_9,          // 6  9
    KEY_KP_ADD,        // 7  +
    KEY_KP_4,          // 8  4
    KEY_KP_5,          // 9  5
    KEY_KP_6,          // 10 6
    KEY_KP_1,          // 11 1
    KEY_KP_2,          // 12 2
    KEY_KP_3,          // 13 3
    KEY_KP_ENTER,      // 14 Enter
    KEY_KP_0,          // 15 0
    KEY_KP_DECIMAL,    // 16 .
    0, 0, 0,           // 17-19 unmapped
    KEY_F13,           // 20 Macro key 1
    KEY_F14,           // 21 Macro key 2
    KEY_F15,           // 22 Macro key 3
    KEY_F16            // 23 Macro key 4
};

// Track previous key states for press/release
uint32_t prevTouched = 0;

// Long-press detection for macro keys (sensors 20-23)
uint32_t macroKeyPressTime[4] = {0, 0, 0, 0};
const uint32_t LONG_PRESS_MS = 1000;

void setup() {
  Serial.begin(115200);

  // Initialize BLE Keyboard
  BLEKeyboard.begin();

  delay(500);
  Serial.println("MPR121 HID keyboard start (BLE)");

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(50);

  if (!mpr1.begin(MPR121_ADDR1)) {
    Serial.println("Error: MPR121 @0x5A not found");
  } else {
    Serial.println("Found MPR121 @0x5A");
  }
  if (!mpr2.begin(MPR121_ADDR2)) {
    Serial.println("Error: MPR121 @0x5B not found");
  } else {
    Serial.println("Found MPR121 @0x5B");
  }
}

void loop() {
  // Update LED animations
  BLEKeyboard.updateLEDs();
  
  // Monitor connection status
  static bool lastConnected = false;
  bool connected = BLEKeyboard.isConnected();
  if (connected != lastConnected) {
    lastConnected = connected;
    Serial.printf("BLE connection changed: %s\n", connected ? "CONNECTED" : "DISCONNECTED");
  }
  
  // Handle touch input less frequently
  static uint32_t lastTouch = 0;
  if (millis() - lastTouch < 20) return;
  lastTouch = millis();
  
  // Build current touched bitmask (24 bits: mpr1[0..11] + mpr2[0..11])
  uint32_t curTouched = 0;

  for (uint8_t i = 0; i < 12; i++) {
    if (mpr1.filteredData(i) < btnbaseline[i]) {
      curTouched |= (1UL << i);
    }
  }
  for (uint8_t i = 0; i < 12; i++) {
    if (mpr2.filteredData(i) < btnbaseline[12 + i]) {
      curTouched |= (1UL << (12 + i));
    }
  }

  // Check long-press on macro keys (sensors 20-23) for reconnect
  for (uint8_t i = 20; i < 24; i++) {
    uint8_t macroIdx = i - 20;
    
    if (curTouched & (1UL << i)) {
      // Key is pressed
      if (macroKeyPressTime[macroIdx] == 0) {
        // Just pressed, record time
        macroKeyPressTime[macroIdx] = millis();
      } else {
        // Check if long-press threshold reached
        if (millis() - macroKeyPressTime[macroIdx] >= LONG_PRESS_MS) {
          Serial.printf("Long press detected on macro key %d, reconnecting...\n", macroIdx);
          BLEKeyboard.disconnect();
          // Reset all press times to avoid repeated triggers
          for (int j = 0; j < 4; j++) {
            macroKeyPressTime[j] = 0;
          }
          break; // Exit loop after triggering reconnect
        }
      }
    } else {
      // Key released, reset timer
      macroKeyPressTime[macroIdx] = 0;
    }
  }

  // Detect press/release
  uint32_t justPressed = curTouched & ~prevTouched;
  uint32_t justReleased = ~curTouched & prevTouched;

  for (uint8_t i = 0; i < 24; i++) {
    uint8_t hid = hidmap[i];
    if (hid == 0) continue;

    if (justPressed & (1UL << i)) {
      if (BLEKeyboard.isConnected()) {
        BLEKeyboard.press(hid);
        Serial.printf("[CONNECTED] Key %d pressed (HID 0x%02X)\n", i, hid);
      } else {
        Serial.printf("[NOT CONNECTED] Key %d pressed ignored\n", i);
      }
    }
    if (justReleased & (1UL << i)) {
      if (BLEKeyboard.isConnected()) {
        BLEKeyboard.release(hid);
        Serial.printf("[CONNECTED] Key %d released (HID 0x%02X)\n", i, hid);
      } else {
        Serial.printf("[NOT CONNECTED] Key %d released ignored\n", i);
      }
    }
  }

  prevTouched = curTouched;

  // Debug print every 2 seconds
  static uint32_t lastDebug = 0;
  if (millis() - lastDebug > 2000) {
    lastDebug = millis();
    Serial.printf("\n=== Status: %s ===\n", BLEKeyboard.isConnected() ? "CONNECTED" : "DISCONNECTED");
    Serial.print("Touched: ");
    for (uint8_t i = 0; i < 24; i++) {
      Serial.print((curTouched & (1UL << i)) ? '1' : '0');
      if (i == 11) Serial.print('|');
    }
    Serial.println();
    
    // Print sensor values for first few electrodes
    Serial.print("Sensor values (first 6): ");
    for (uint8_t i = 0; i < 6; i++) {
      uint16_t val = mpr1.filteredData(i);
      Serial.printf("%d(%d) ", val, btnbaseline[i]);
    }
    Serial.println();
  }
}