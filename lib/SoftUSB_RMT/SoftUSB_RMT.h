/**
 * SoftUSB_RMT - Software Low-Speed USB using RMT peripheral
 * 
 * Uses ESP32-C3's RMT peripheral for precise timing:
 * - RMT RX to capture incoming USB packets
 * - RMT TX to send USB packets with precise timing
 * 
 * USB Low-Speed: 1.5 Mbps = 666.67ns per bit
 * RMT resolution: 12.5ns at 80MHz = 53.33 RMT ticks per USB bit
 */

#ifndef SOFTUSB_RMT_H
#define SOFTUSB_RMT_H

#include <Arduino.h>
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"

// USB PIDs
#define USB_PID_OUT     0xE1
#define USB_PID_IN      0x69
#define USB_PID_SETUP   0x2D
#define USB_PID_DATA0   0xC3
#define USB_PID_DATA1   0x4B
#define USB_PID_ACK     0xD2
#define USB_PID_NAK     0x5A
#define USB_PID_STALL   0x1E

// USB Descriptor Types
#define USB_DESC_DEVICE        1
#define USB_DESC_CONFIGURATION 2
#define USB_DESC_STRING        3
#define USB_DESC_INTERFACE     4
#define USB_DESC_ENDPOINT      5
#define USB_DESC_HID           0x21
#define USB_DESC_HID_REPORT    0x22

// HID Keyboard keycodes
#define KEY_NONE        0x00
#define KEY_NUM_LOCK    0x53
#define KEY_KP_SLASH    0x54
#define KEY_KP_ASTERISK 0x55
#define KEY_KP_MINUS    0x56
#define KEY_KP_7        0x5F
#define KEY_KP_8        0x60
#define KEY_KP_9        0x61
#define KEY_KP_PLUS     0x57
#define KEY_KP_4        0x5C
#define KEY_KP_5        0x5D
#define KEY_KP_6        0x5E
#define KEY_KP_1        0x59
#define KEY_KP_2        0x5A
#define KEY_KP_3        0x5B
#define KEY_KP_ENTER    0x58
#define KEY_KP_0        0x62
#define KEY_KP_DOT      0x63

// RMT timing constants for USB Low-Speed
// At 80MHz RMT clock: 1 tick = 12.5ns
// USB bit time = 666.67ns = 53.33 ticks
#define RMT_USB_BIT_TICKS   53
#define RMT_USB_HALF_BIT    27

// Buffer sizes
#define USB_RX_BUF_SIZE     64
#define RMT_RX_BUF_SIZE     256

class SoftUSBKeyboard_RMT {
public:
    SoftUSBKeyboard_RMT(uint8_t pinDp, uint8_t pinDm);
    
    bool begin();
    void task();
    
    // Keyboard functions
    void pressKey(uint8_t keycode);
    void releaseKey(uint8_t keycode);
    void releaseAll();
    
private:
    uint8_t _pinDp;
    uint8_t _pinDm;
    
    // USB state
    bool _configured;
    uint8_t _address;
    uint8_t _pendingAddress;  // Address to set after status stage
    bool _dataToggle;
    bool _rxEnabled;
    
    // Pending response data (for IN tokens)
    const uint8_t* _txData;
    uint8_t _txLen;
    uint8_t _txOffset;
    
    // HID report
    uint8_t _keyReport[8];
    bool _reportChanged;
    
    // RMT handles
    rmt_channel_handle_t _rxChannel;
    rmt_channel_handle_t _txChannel;
    rmt_encoder_handle_t _txEncoder;
    
    // RX buffer
    rmt_symbol_word_t _rxBuffer[RMT_RX_BUF_SIZE];
    
    // For multi-packet decode
    int _lastBitPos;
    uint8_t _lastBitLevel;
    
    // Internal functions
    bool initRMT();
    void setOutput();
    void setInput();
    
    // USB protocol
    bool startReceive();
    bool checkReceive(uint8_t* buffer, uint8_t* len);
    bool receivePacket(uint8_t* buffer, uint8_t* len, uint32_t timeoutMs);
    void sendBit(bool dpLevel, bool dmLevel);
    void sendPacket(const uint8_t* data, uint8_t len);
    void sendHandshake(uint8_t pid);
    void sendDataPacket(uint8_t pid, const uint8_t* data, uint8_t len);
    
    // Encoding/Decoding
    void encodeNRZI(const uint8_t* data, uint8_t len, rmt_symbol_word_t* symbols, size_t* symbolCount);
    bool decodeNRZI(const rmt_symbol_word_t* symbols, size_t symbolCount, uint8_t* data, uint8_t* len);
    bool decodeNextPacket(uint8_t* data, uint8_t* len);
    
    uint8_t crc5(uint16_t data);
    uint16_t crc16(const uint8_t* data, uint8_t len);
    
    void handleSetupPacket(const uint8_t* setup);
    void handleGetDescriptor(uint8_t type, uint8_t index, uint16_t length);
};

#endif // SOFTUSB_RMT_H
