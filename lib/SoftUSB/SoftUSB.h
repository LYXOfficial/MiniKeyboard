/**
 * SoftUSB - Software Low-Speed USB HID Keyboard Implementation
 * 
 * Implements USB Low-Speed (1.5Mbps) HID keyboard using GPIO bit-banging.
 * Designed for ESP32-C3 which lacks USB OTG hardware.
 */

#ifndef SOFTUSB_H
#define SOFTUSB_H

#include <Arduino.h>

// USB PID (Packet Identifier) definitions
#define USB_PID_OUT     0xE1
#define USB_PID_IN      0x69
#define USB_PID_SOF     0xA5
#define USB_PID_SETUP   0x2D
#define USB_PID_DATA0   0xC3
#define USB_PID_DATA1   0x4B
#define USB_PID_ACK     0xD2
#define USB_PID_NAK     0x5A
#define USB_PID_STALL   0x1E

// USB Descriptor Types
#define USB_DESC_DEVICE         0x01
#define USB_DESC_CONFIGURATION  0x02
#define USB_DESC_STRING         0x03
#define USB_DESC_INTERFACE      0x04
#define USB_DESC_ENDPOINT       0x05
#define USB_DESC_HID            0x21
#define USB_DESC_HID_REPORT     0x22

// USB Standard Requests
#define USB_REQ_GET_STATUS        0x00
#define USB_REQ_CLEAR_FEATURE     0x01
#define USB_REQ_SET_FEATURE       0x03
#define USB_REQ_SET_ADDRESS       0x05
#define USB_REQ_GET_DESCRIPTOR    0x06
#define USB_REQ_SET_DESCRIPTOR    0x07
#define USB_REQ_GET_CONFIGURATION 0x08
#define USB_REQ_SET_CONFIGURATION 0x09

// HID Class Requests
#define HID_REQ_GET_REPORT    0x01
#define HID_REQ_GET_IDLE      0x02
#define HID_REQ_GET_PROTOCOL  0x03
#define HID_REQ_SET_REPORT    0x09
#define HID_REQ_SET_IDLE      0x0A
#define HID_REQ_SET_PROTOCOL  0x0B

// HID Keyboard Keycodes (Numpad)
#define HID_KEY_NUM_LOCK    0x53
#define HID_KEY_KP_DIVIDE   0x54
#define HID_KEY_KP_MULTIPLY 0x55
#define HID_KEY_KP_SUBTRACT 0x56
#define HID_KEY_KP_ADD      0x57
#define HID_KEY_KP_ENTER    0x58
#define HID_KEY_KP_1        0x59
#define HID_KEY_KP_2        0x5A
#define HID_KEY_KP_3        0x5B
#define HID_KEY_KP_4        0x5C
#define HID_KEY_KP_5        0x5D
#define HID_KEY_KP_6        0x5E
#define HID_KEY_KP_7        0x5F
#define HID_KEY_KP_8        0x60
#define HID_KEY_KP_9        0x61
#define HID_KEY_KP_0        0x62
#define HID_KEY_KP_DECIMAL  0x63

class SoftUSBKeyboard {
public:
    SoftUSBKeyboard(uint8_t pinDp, uint8_t pinDm);
    
    bool begin();
    void task();  // Must be called frequently in loop()
    
    void pressKey(uint8_t keycode);
    void releaseKey(uint8_t keycode);
    void releaseAll();
    
    bool isReady() { return _configured; }
    
private:
    uint8_t _pinDp;
    uint8_t _pinDm;
    bool _configured;
    uint8_t _address;
    bool _dataToggle;
    
    // HID report: [modifier, reserved, key1, key2, key3, key4, key5, key6]
    uint8_t _keyReport[8];
    bool _reportChanged;
    
    // Low-level signal generation
    inline void setDP(bool high);
    inline void setDM(bool high);
    inline bool getDP();
    inline bool getDM();
    inline void setOutput();
    inline void setInput();
    
    void delayBit();
    void delayHalfBit();
    
    void sendSE0(uint16_t durationUs);
    void sendJ();
    void sendK();
    void sendBit(uint8_t bit);
    void sendByte(uint8_t byte);
    void sendPacket(const uint8_t* data, uint8_t len);
    void sendHandshake(uint8_t pid);
    void sendDataPacket(uint8_t pid, const uint8_t* data, uint8_t len);
    
    bool waitForPacket(uint8_t* buffer, uint8_t* len, uint32_t timeoutUs);
    
    // Protocol handling
    void handleSetupPacket(const uint8_t* setup);
    void handleGetDescriptor(uint8_t type, uint8_t index, uint16_t length);
    void handleHIDRequest(const uint8_t* setup);
    
    // CRC calculation
    uint8_t crc5(uint16_t data);
    uint16_t crc16(const uint8_t* data, uint8_t len);
};

#endif // SOFTUSB_H
