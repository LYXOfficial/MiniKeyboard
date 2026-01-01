#include "BLEKeyboard.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>
#include <HIDKeyboardTypes.h>

// NimBLE includes for bonding management
#include "sdkconfig.h"
#if defined(CONFIG_BT_NIMBLE_ENABLED)
#include <host/ble_hs.h>
#include <host/util/util.h>
#elif defined(CONFIG_BLUEDROID_ENABLED)
#include <esp_gap_ble_api.h>
#endif

// Macro LED pins
#define LED_MACRO_0  4
#define LED_MACRO_1  5
#define LED_MACRO_2  6
#define LED_MACRO_3  7
#define LED_NUMLOCK  3

// Animation timing
#define BOOT_INTERVAL_MS    50
#define BLINK_INTERVAL_MS   50
#define BREATH_PERIOD_MS    2000

// Basic keyboard report: 8 bytes (modifier, reserved, 6 keycodes)
static uint8_t _report[8];

static BLEServer* pServer = nullptr;
static BLEHIDDevice* hid = nullptr;
static BLECharacteristic* inputReport = nullptr;
static BLECharacteristic* outputReport = nullptr; // host -> device (LEDs)
static bool _connected = false;
static bool _wasConnected = false;

BLEKeyboardClass BLEKeyboard("AriaKeyboard", "Aria");

// Server callbacks for connection events
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    _connected = true;
    Serial.println("BLE connected");
  }
  void onDisconnect(BLEServer* s) override {
    _connected = false;
    Serial.println("BLE disconnected");
    // Delay before restarting advertising to allow cleanup
    delay(100);
    // Restart advertising
    BLEDevice::startAdvertising();
  }
};

BLEKeyboardClass::BLEKeyboardClass(const char* name, const char* manufacturer) 
  : _started(false), _name(name), _manufacturer(manufacturer), _ledState(LED_OFF), _ledTimer(0), _ledStep(0) {
}

void BLEKeyboardClass::_initMacroLEDs() {
  // Use analogWrite (LEDC) for smooth breathing
  // No need for pinMode(OUTPUT) when using analogWrite on ESP32
  _setMacroLEDs(false, false, false, false);
}

void BLEKeyboardClass::_setMacroLEDs(bool led4, bool led5, bool led6, bool led7) {
  analogWrite(LED_MACRO_0, led4 ? 255 : 0);
  analogWrite(LED_MACRO_1, led5 ? 255 : 0);
  analogWrite(LED_MACRO_2, led6 ? 255 : 0);
  analogWrite(LED_MACRO_3, led7 ? 255 : 0);
}

// Helper for hardware PWM brightness
static void _setMacroBrightness(uint8_t brightness) {
  analogWrite(LED_MACRO_0, brightness);
  analogWrite(LED_MACRO_1, brightness);
  analogWrite(LED_MACRO_2, brightness);
  analogWrite(LED_MACRO_3, brightness);
}

bool BLEKeyboardClass::begin() {
  if (_started) return true;

  // Initialize macro LEDs first - all off
  _initMacroLEDs();
  _ledState = LED_BOOT_RUNNING;
  _ledTimer = millis();
  _ledStep = 0;

  BLEDevice::init(_name);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  
  hid = new BLEHIDDevice(pServer);

  inputReport = hid->inputReport(1); // report ID 1
  hid->manufacturer()->setValue(_manufacturer);
  hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
  hid->hidInfo(0x00, 0x01);

  // Standard keyboard HID report descriptor
  // Usage Maximum extended to 0xFF to support F13-F24 and other extended keys
  static const uint8_t reportMap[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       // REPORT_ID (1) - must match inputReport(1)
    0x05, 0x07,       // Usage Page (Key Codes)
    0x19, 0xE0,       // Usage Minimum (224)
    0x29, 0xE7,       // Usage Maximum (231)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x01,       // Logical Maximum (1)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x08,       // Report Count (8)
    0x81, 0x02,       // Input (Data, Variable, Absolute) ; Modifier byte
    0x95, 0x01,       // Report Count (1)
    0x75, 0x08,       // Report Size (8)
    0x81, 0x01,       // Input (Constant) ; Reserved byte
    0x95, 0x06,       // Report Count (6)
    0x75, 0x08,       // Report Size (8)
    0x15, 0x00,       // Logical Minimum (0)
    0x26, 0xFF, 0x00, // Logical Maximum (255) - extended for F13-F24
    0x05, 0x07,       // Usage Page (Key Codes)
    0x19, 0x00,       // Usage Minimum (0)
    0x2A, 0xFF, 0x00, // Usage Maximum (255) - extended for F13-F24
    0x81, 0x00,       // Input (Data, Array) ; Key codes
    0xC0              // End Collection
  };

  hid->reportMap((uint8_t*)reportMap, sizeof(reportMap));

  // HID requires encrypted connection - use bonding with Just Works
  BLESecurity* pSecurity = new BLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);
  pSecurity->setCapability(ESP_IO_CAP_NONE);
  pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  // Start all HID services including Device Info and Battery
  hid->startServices();
  
  // Set battery level (required for proper HID initialization)
  hid->setBatteryLevel(100);

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->setAppearance(0x03C1); // Generic HID
  pAdv->addServiceUUID(hid->hidService()->getUUID());
  
  // PC discovery improvements
  pAdv->setScanResponse(false); // 尝试关闭扫描响应，将所有信息放入主广播包
  pAdv->setMinPreferred(0x06);  
  pAdv->setMaxPreferred(0x12);
  
  pAdv->start(); // 使用实例方法启动广播

  memset(_report, 0, sizeof(_report));
  // Configure NumLock LED pin (IO3): high = on, low = off
  pinMode(LED_NUMLOCK, OUTPUT);
  digitalWrite(LED_NUMLOCK, LOW);

  // Get output report (LED state) and attach write callback
  outputReport = hid->outputReport(1);
  if (outputReport) {
    class OutCB : public BLECharacteristicCallbacks {
    public:
      void onWrite(BLECharacteristic* chr) override {
        std::string val = std::string(chr->getValue().c_str());
        if (val.length() >= 1) {
          uint8_t leds = (uint8_t)val[0];
          // Bit 0 = Num Lock per HID LED usage
          if (leds & 0x01) {
            digitalWrite(LED_NUMLOCK, HIGH);
          } else {
            digitalWrite(LED_NUMLOCK, LOW);
          }
        }
      }
    };
    outputReport->setCallbacks(new OutCB());
  }
  _started = true;
  return true;
}

void BLEKeyboardClass::end() {
  if (!_started) return;
  // No explicit stopServices in this BLEHIDDevice variant; just deinit BLE
  BLEDevice::deinit(true);
  _started = false;
}

bool BLEKeyboardClass::isConnected() {
  if (!pServer) return false;
  return pServer->getConnectedCount() > 0;
}

void BLEKeyboardClass::press(uint8_t key) {
  if (!isConnected()) {
    Serial.println("[BLE] press() called but not connected");
    return;
  }
  for (int i = 2; i < 8; i++) {
    if (_report[i] == 0) {
      _report[i] = key;
      break;
    }
  }
  if (inputReport) {
    inputReport->setValue(_report, sizeof(_report));
    inputReport->notify();
    Serial.printf("[BLE] Sent report: %02X %02X %02X %02X %02X %02X %02X %02X\n",
      _report[0], _report[1], _report[2], _report[3],
      _report[4], _report[5], _report[6], _report[7]);
  } else {
    Serial.println("[BLE] inputReport is NULL!");
  }
}

void BLEKeyboardClass::release(uint8_t key) {
  if (!isConnected()) return;
  for (int i = 2; i < 8; i++) {
    if (_report[i] == key) {
      _report[i] = 0;
      break;
    }
  }
  if (inputReport) {
    inputReport->setValue(_report, sizeof(_report));
    inputReport->notify();
  }
}

void BLEKeyboardClass::releaseAll() {
  if (!isConnected()) return;
  memset(_report, 0, sizeof(_report));
  if (inputReport) {
    inputReport->setValue(_report, sizeof(_report));
    inputReport->notify();
  }
}

void BLEKeyboardClass::sendReport(uint8_t* report, size_t len) {
  if (!inputReport) return;
  inputReport->setValue(report, len);
  inputReport->notify();
}

// Check if any bonded devices exist
bool BLEKeyboardClass::hasBondedDevices() {
#if defined(CONFIG_BT_NIMBLE_ENABLED)
  int count = 0;
  ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_SEC, &count);
  return count > 0;
#elif defined(CONFIG_BLUEDROID_ENABLED)
  int count = esp_ble_get_bond_device_num();
  return count > 0;
#else
  return false;
#endif
}

// Clear all bonding info
void BLEKeyboardClass::clearAllBonding() {
  Serial.println("Executing clearAllBonding...");
#if defined(CONFIG_BT_NIMBLE_ENABLED)
  int rc = ble_store_clear();
  Serial.printf("NimBLE store clear result: %d\n", rc);
#elif defined(CONFIG_BLUEDROID_ENABLED)
  int count = esp_ble_get_bond_device_num();
  if (count > 0) {
    esp_ble_bond_dev_t* list = (esp_ble_bond_dev_t*)malloc(count * sizeof(esp_ble_bond_dev_t));
    if (list) {
      esp_ble_get_bond_device_list(&count, list);
      for (int i = 0; i < count; i++) {
        esp_ble_remove_bond_device(list[i].bd_addr);
      }
      free(list);
    }
  }
#endif
  Serial.println("All bonding info cleared");
}

// Disconnect current connection
void BLEKeyboardClass::disconnect() {
  if (pServer && _connected) {
    // Get connection handle and disconnect
    pServer->disconnect(pServer->getConnId());
    _connected = false;
  }
}

// Restart advertising
void BLEKeyboardClass::restartAdvertising() {
  BLEDevice::startAdvertising();
}

// LED state management
void BLEKeyboardClass::setLEDState(LEDState state) {
  if (_ledState != state) {
    _ledState = state;
    _ledTimer = millis();
    _ledStep = 0;
    if (state == LED_CONNECTED || state == LED_OFF) {
      _setMacroLEDs(false, false, false, false);
    }
  }
}

LEDState BLEKeyboardClass::getLEDState() {
  return _ledState;
}

// Running lights for boot sequence
void BLEKeyboardClass::_runBootSequence() {
  uint32_t now = millis();
  if (now - _ledTimer >= BOOT_INTERVAL_MS) {
    _ledTimer = now;
    // Turn on one LED at a time: 0->1->2->3
    _setMacroLEDs(_ledStep == 0, _ledStep == 1, _ledStep == 2, _ledStep == 3);
    _ledStep++;
    if (_ledStep >= 4) {
      _setMacroLEDs(false, false, false, false);
      // Boot sequence done, switch to next state
      if (hasBondedDevices()) {
        _ledState = LED_BLINK_CONNECTING;
      } else {
        _ledState = LED_BREATH_PAIRING;
      }
      _ledTimer = now;
      _ledStep = 0;
    }
  }
}

// Blinking animation (all LEDs on/off simultaneously)
void BLEKeyboardClass::_runBlinkAnimation() {
  uint32_t now = millis();
  if (now - _ledTimer >= BLINK_INTERVAL_MS) {
    _ledTimer = now;
    _ledStep = !_ledStep;
    bool on = (_ledStep == 1);
    _setMacroLEDs(on, on, on, on);
  }
}

// Breathing animation using sine-like curve with hardware PWM
void BLEKeyboardClass::_runBreathAnimation() {
  uint32_t now = millis();
  uint32_t elapsed = (now - _ledTimer) % BREATH_PERIOD_MS;
  
  // Calculate brightness 0-255 using sine approximation
  float phase = (float)elapsed / BREATH_PERIOD_MS * 2.0 * PI;
  // Sine wave shifted to 0-255 range
  int brightness = (int)((sin(phase - PI/2) + 1.0) * 127.5);
  
  _setMacroBrightness(brightness);
}

// Call this in main loop to update LED animations
void BLEKeyboardClass::updateLEDs() {
  // Check for connection state changes
  if (_connected && !_wasConnected) {
    // Just connected
    setLEDState(LED_CONNECTED);
  } else if (!_connected && _wasConnected) {
    // Just disconnected
    if (hasBondedDevices()) {
      setLEDState(LED_BLINK_CONNECTING);
    } else {
      setLEDState(LED_BREATH_PAIRING);
    }
  }
  _wasConnected = _connected;
  
  // Run animation based on current state
  switch (_ledState) {
    case LED_BOOT_RUNNING:
      _runBootSequence();
      break;
    case LED_BLINK_CONNECTING:
      _runBreathAnimation();
      break;
    case LED_BREATH_PAIRING:
      _runBreathAnimation();
      break;
    case LED_CONNECTED:
    case LED_OFF:
    default:
      // LEDs already off
      break;
  }
}
