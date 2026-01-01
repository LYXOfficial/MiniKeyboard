#ifndef BLEKEYBOARD_H
#define BLEKEYBOARD_H

#include <Arduino.h>

// LED animation states
enum LEDState {
  LED_OFF,
  LED_BOOT_RUNNING,     // Running lights during boot
  LED_BLINK_CONNECTING, // Blinking while connecting (has pairing)
  LED_BREATH_PAIRING,   // Breathing while waiting for pairing (no paired devices)
  LED_CONNECTED         // All off when connected
};

class BLEKeyboardClass {
public:
  BLEKeyboardClass(const char* name = "AriaKeyboard", const char* manufacturer = "Aria");
  bool begin();
  void end();
  bool isConnected();
  bool hasBondedDevices();
  void press(uint8_t key);
  void release(uint8_t key);
  void releaseAll();
  void sendReport(uint8_t* report, size_t len);
  
  // LED animation control
  void updateLEDs();       // Call in main loop
  void setLEDState(LEDState state);
  LEDState getLEDState();
  
  // Pairing management
  void clearAllBonding();  // Forget all paired devices
  void disconnect();       // Disconnect current connection
  void restartAdvertising();
  
private:
  bool _started;
  const char* _name;
  const char* _manufacturer;
  LEDState _ledState;
  uint32_t _ledTimer;
  uint8_t _ledStep;
  
  void _initMacroLEDs();
  void _setMacroLEDs(bool led4, bool led5, bool led6, bool led7);
  void _runBootSequence();
  void _runBlinkAnimation();
  void _runBreathAnimation();
};

extern BLEKeyboardClass BLEKeyboard;

#endif
