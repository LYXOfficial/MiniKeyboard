/**
 * SoftUSB_GPIO - Software Low-Speed USB using GPIO polling in ISR
 * 
 * This implementation handles USB timing requirements by doing
 * all packet RX/TX in interrupt context using tight polling loops.
 * 
 * USB Low-Speed timing requirements:
 * - Bit time: 666.67 ns (1.5 Mbps)
 * - Device response timeout: 7.5 µs (max 18 bit times after EOP)
 */

#ifndef SOFT_USB_GPIO_H
#define SOFT_USB_GPIO_H

#include <Arduino.h>

// USB PIDs
#define USB_PID_OUT     0xE1
#define USB_PID_IN      0x69
#define USB_PID_SETUP   0x2D
#define USB_PID_DATA0   0xC3
#define USB_PID_DATA1   0x4B
#define USB_PID_ACK     0xD2
#define USB_PID_NAK     0x5A
#define USB_PID_STALL   0x1E
#define USB_PID_SOF     0xA5

// Descriptor types
#define USB_DESC_DEVICE        1
#define USB_DESC_CONFIGURATION 2
#define USB_DESC_STRING        3
#define USB_DESC_INTERFACE     4
#define USB_DESC_ENDPOINT      5
#define USB_DESC_HID           0x21
#define USB_DESC_HID_REPORT    0x22

// USB control states
#define USB_STATE_IDLE        0
#define USB_STATE_DATA_IN     1
#define USB_STATE_DATA_OUT    2
#define USB_STATE_STATUS      3

class SoftUSBKeyboard_GPIO {
public:
    SoftUSBKeyboard_GPIO(int pinDp, int pinDm);
    
    bool begin();
    void task();
    void pressKey(uint8_t key);
    void releaseKey(uint8_t key);
    void releaseAll();
    bool isReady();
    
private:
    int _pinDp;
    int _pinDm;
    uint32_t _dpMask;
    uint32_t _dmMask;
    
    uint8_t _address;
    bool _configured;
    
    // Control transfer state
    uint8_t _ctrlState;
    uint8_t _txBuffer[64];
    uint8_t _txLen;
    uint8_t _txOffset;
    uint8_t _pendingAddress;
    bool _dataPID;  // false = DATA0, true = DATA1
    
    // HID state
    uint8_t _keyReport[8];  // Modifier, Reserved, Keys[6]
    bool _reportChanged;
    
    // ISR handler
    static void IRAM_ATTR gpioISR(void* arg);
    void IRAM_ATTR handleUSB();
    
    // USB primitives (all in IRAM)
    int IRAM_ATTR receivePacket(uint8_t* buffer, int maxLen);
    void IRAM_ATTR sendPacket(const uint8_t* data, int len);
    void IRAM_ATTR sendHandshake(uint8_t pid);
    void IRAM_ATTR sendDataPacket(uint8_t pid, const uint8_t* data, int len);
    
    // USB handlers
    void handleSetupPacket(const uint8_t* setup);
    void handleGetDescriptor(uint8_t descType, uint8_t descIndex, uint16_t wLength);
    
    // CRC
    static uint8_t crc5(uint16_t data);
    static uint16_t crc16(const uint8_t* data, int len);
    
    // Timing
    static inline void IRAM_ATTR delayBit();
    static inline void IRAM_ATTR delayHalfBit();
};

#endif
