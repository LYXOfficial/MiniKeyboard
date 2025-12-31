#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPR121.h>
#include "SoftUSB.h"

#define SDA_PIN 2
#define SCL_PIN 1
#define MPR121_ADDR1 0x5A
#define MPR121_ADDR2 0x5B

// SoftUSB pins - D+ on GPIO19, D- on GPIO18
#define USB_DP_PIN 19
#define USB_DM_PIN 18

Adafruit_MPR121 mpr1 = Adafruit_MPR121();
Adafruit_MPR121 mpr2 = Adafruit_MPR121();

SoftUSBKeyboard Keyboard(USB_DP_PIN, USB_DM_PIN);

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
    0, 0, 0, 0, 0, 0, 0 // remaining unmapped
};

// Track previous key states for press/release
uint32_t prevTouched = 0;

void setup() {
  Serial.begin(115200);

  // Initialize SoftUSB Keyboard
  Keyboard.begin();

  delay(500);
  Serial.println("MPR121 HID keyboard start (SoftUSB)");

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

  // Detect press/release
  uint32_t justPressed = curTouched & ~prevTouched;
  uint32_t justReleased = ~curTouched & prevTouched;

  for (uint8_t i = 0; i < 24; i++) {
    uint8_t hid = hidmap[i];
    if (hid == 0) continue;

    if (justPressed & (1UL << i)) {
      Keyboard.pressKey(hid);
      Serial.printf("Key %d pressed (HID 0x%02X)\n", i, hid);
    }
    if (justReleased & (1UL << i)) {
      Keyboard.releaseKey(hid);
      Serial.printf("Key %d released (HID 0x%02X)\n", i, hid);
    }
  }

  prevTouched = curTouched;

  // Run SoftUSB task to handle USB protocol
  Keyboard.task();

  // Debug print every 500ms
  static uint32_t lastDebug = 0;
  if (millis() - lastDebug > 500) {
    lastDebug = millis();
    Serial.print("touched: ");
    for (uint8_t i = 0; i < 24; i++) {
      Serial.print((curTouched & (1UL << i)) ? '1' : '0');
      if (i == 11) Serial.print('|');
    }
    Serial.println();
  }

  delay(10);
}