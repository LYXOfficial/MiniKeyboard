/**
 * SoftUSB_GPIO - Software Low-Speed USB using GPIO polling in ISR
 * 
 * All USB handling is done in interrupt context to meet timing requirements.
 * USB Low-Speed requires response within 7.5 µs of packet end.
 */

#include "SoftUSB_GPIO.h"
#include "soc/gpio_struct.h"
#include "soc/usb_serial_jtag_struct.h"
#include "esp_cpu.h"
#include "driver/gpio.h"

// CPU cycles per USB bit at 160 MHz (666.67 ns)
#define CYCLES_PER_BIT 107
#define CYCLES_HALF_BIT 53

// Macros for direct GPIO access (faster than function calls)
#define GPIO_IN (GPIO.in.val)
#define GPIO_OUT_SET (GPIO.out_w1ts.val)
#define GPIO_OUT_CLR (GPIO.out_w1tc.val)

// USB Device Descriptor
static const uint8_t DRAM_ATTR deviceDescriptor[] = {
    18,                 // bLength
    USB_DESC_DEVICE,    // bDescriptorType
    0x10, 0x01,         // bcdUSB (USB 1.1)
    0x00,               // bDeviceClass
    0x00,               // bDeviceSubClass
    0x00,               // bDeviceProtocol
    8,                  // bMaxPacketSize0
    0x34, 0x12,         // idVendor
    0xCD, 0xAB,         // idProduct
    0x00, 0x01,         // bcdDevice
    1,                  // iManufacturer
    2,                  // iProduct
    0,                  // iSerialNumber
    1                   // bNumConfigurations
};

// HID Report Descriptor
static const uint8_t DRAM_ATTR hidReportDescriptor[] = {
    0x05, 0x01,  // Usage Page (Generic Desktop)
    0x09, 0x06,  // Usage (Keyboard)
    0xA1, 0x01,  // Collection (Application)
    0x05, 0x07,  //   Usage Page (Key Codes)
    0x19, 0xE0,  //   Usage Minimum (224)
    0x29, 0xE7,  //   Usage Maximum (231)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x01,  //   Logical Maximum (1)
    0x75, 0x01,  //   Report Size (1)
    0x95, 0x08,  //   Report Count (8)
    0x81, 0x02,  //   Input (Data, Variable, Absolute)
    0x95, 0x01,  //   Report Count (1)
    0x75, 0x08,  //   Report Size (8)
    0x81, 0x01,  //   Input (Constant)
    0x95, 0x06,  //   Report Count (6)
    0x75, 0x08,  //   Report Size (8)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x65,  //   Logical Maximum (101)
    0x05, 0x07,  //   Usage Page (Key Codes)
    0x19, 0x00,  //   Usage Minimum (0)
    0x29, 0x65,  //   Usage Maximum (101)
    0x81, 0x00,  //   Input (Data, Array)
    0xC0         // End Collection
};

// Configuration Descriptor (full)
static const uint8_t DRAM_ATTR configDescriptor[] = {
    // Configuration
    9, USB_DESC_CONFIGURATION,
    34, 0,              // wTotalLength
    1,                  // bNumInterfaces
    1,                  // bConfigurationValue
    0,                  // iConfiguration
    0x80,               // bmAttributes (bus powered)
    50,                 // bMaxPower (100mA)
    
    // Interface
    9, USB_DESC_INTERFACE,
    0,                  // bInterfaceNumber
    0,                  // bAlternateSetting
    1,                  // bNumEndpoints
    0x03,               // bInterfaceClass (HID)
    0x01,               // bInterfaceSubClass (Boot)
    0x01,               // bInterfaceProtocol (Keyboard)
    0,                  // iInterface
    
    // HID Descriptor
    9, USB_DESC_HID,
    0x11, 0x01,         // bcdHID
    0,                  // bCountryCode
    1,                  // bNumDescriptors
    USB_DESC_HID_REPORT,
    sizeof(hidReportDescriptor), 0,
    
    // Endpoint
    7, USB_DESC_ENDPOINT,
    0x81,               // bEndpointAddress (IN 1)
    0x03,               // bmAttributes (Interrupt)
    8, 0,               // wMaxPacketSize
    10                  // bInterval (10ms)
};

// String descriptors
static const uint8_t DRAM_ATTR stringDesc0[] = {4, USB_DESC_STRING, 0x09, 0x04};
static const uint8_t DRAM_ATTR stringDesc1[] = {14, USB_DESC_STRING, 
    'S',0,'o',0,'f',0,'t',0,'U',0,'S',0};
static const uint8_t DRAM_ATTR stringDesc2[] = {20, USB_DESC_STRING,
    'K',0,'e',0,'y',0,'b',0,'o',0,'a',0,'r',0,'d',0,'s',0};

// Instance pointer for ISR
static SoftUSBKeyboard_GPIO* _instance = nullptr;
static volatile uint32_t _isrCount = 0;
static volatile int _lastRxLen = 0;

SoftUSBKeyboard_GPIO::SoftUSBKeyboard_GPIO(int pinDp, int pinDm)
    : _pinDp(pinDp), _pinDm(pinDm), _address(0), _configured(false),
      _ctrlState(USB_STATE_IDLE), _txLen(0), _txOffset(0), _pendingAddress(0),
      _dataPID(false), _reportChanged(false) {
    _dpMask = 1 << pinDp;
    _dmMask = 1 << pinDm;
    memset(_keyReport, 0, sizeof(_keyReport));
}

bool SoftUSBKeyboard_GPIO::begin() {
    _instance = this;
    
    // Disable USB Serial/JTAG PHY to free GPIO18/19
    USB_SERIAL_JTAG.conf0.usb_pad_enable = 0;
    
    // Configure pins as input initially
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << _pinDp) | (1ULL << _pinDm),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    Serial.printf("SoftUSB_GPIO: D+ on GPIO%d, D- on GPIO%d\n", _pinDp, _pinDm);
    
    // For Low-Speed USB, external 1.5kΩ pull-up on D- signals device presence
    // After attach, idle state is J = D+ low, D- high
    
    delayMicroseconds(100);
    
    int dp = gpio_get_level((gpio_num_t)_pinDp);
    int dm = gpio_get_level((gpio_num_t)_pinDm);
    Serial.printf("Initial: D+=%d D-=%d\n", dp, dm);
    
    // Note: We will poll instead of using interrupts
    // GPIO18/19 on ESP32-C3 are USB pins and may have interrupt limitations
    
    Serial.println("SoftUSB_GPIO: Polling mode");
    
    return true;
}

void IRAM_ATTR SoftUSBKeyboard_GPIO::gpioISR(void* arg) {
    SoftUSBKeyboard_GPIO* self = (SoftUSBKeyboard_GPIO*)arg;
    
    // Disable interrupt to prevent re-entry
    gpio_intr_disable((gpio_num_t)self->_pinDp);
    
    // Handle USB packet
    self->handleUSB();
    
    // Re-enable interrupt
    gpio_intr_enable((gpio_num_t)self->_pinDp);
}

void IRAM_ATTR SoftUSBKeyboard_GPIO::handleUSB() {
    _isrCount++;
    
    uint8_t buffer[16];
    
    // Receive packet (we already caught first edge)
    int len = receivePacket(buffer, sizeof(buffer));
    _lastRxLen = len;
    
    if (len < 1) return;
    
    uint8_t pid = buffer[0];
    
    // Handle different packet types
    switch (pid) {
        case USB_PID_SETUP: {
            // SETUP token - need to receive DATA0 next
            // Wait for DATA0 packet
            uint8_t data[16];
            int dataLen = receivePacket(data, sizeof(data));
            
            if (dataLen >= 9 && data[0] == USB_PID_DATA0) {
                // Send ACK immediately
                sendHandshake(USB_PID_ACK);
                
                // Process SETUP data (stored for later handling)
                handleSetupPacket(&data[1]);
            }
            break;
        }
        
        case USB_PID_IN: {
            // IN token - send data or NAK
            if (_txLen > 0 && _txOffset < _txLen) {
                int remaining = _txLen - _txOffset;
                int toSend = (remaining > 8) ? 8 : remaining;
                
                uint8_t dataPID = _dataPID ? USB_PID_DATA1 : USB_PID_DATA0;
                sendDataPacket(dataPID, &_txBuffer[_txOffset], toSend);
                _dataPID = !_dataPID;
                _txOffset += toSend;
                
                if (_txOffset >= _txLen) {
                    // All data sent, enter status stage
                    _ctrlState = USB_STATE_STATUS;
                }
            } else if (_reportChanged) {
                // Send HID report on EP1
                sendDataPacket(USB_PID_DATA1, _keyReport, 8);
                _reportChanged = false;
            } else {
                sendHandshake(USB_PID_NAK);
            }
            break;
        }
        
        case USB_PID_OUT: {
            // OUT token - receive data
            uint8_t data[16];
            int dataLen = receivePacket(data, sizeof(data));
            
            if (dataLen >= 1) {
                sendHandshake(USB_PID_ACK);
                
                // Status stage complete
                if (_ctrlState == USB_STATE_STATUS) {
                    if (_pendingAddress != 0) {
                        _address = _pendingAddress;
                        _pendingAddress = 0;
                    }
                    _ctrlState = USB_STATE_IDLE;
                    _txLen = 0;
                    _txOffset = 0;
                }
            }
            break;
        }
        
        case USB_PID_SOF:
            // Ignore SOF packets
            break;
    }
}

// Receive packet using tight polling loop
// Returns number of bytes received (including PID)
int IRAM_ATTR SoftUSBKeyboard_GPIO::receivePacket(uint8_t* buffer, int maxLen) {
    uint32_t dpMask = _dpMask;
    volatile uint32_t* gpio_in = &GPIO.in.val;
    
    // Wait for SYNC pattern end (after KK = D+ high for 2 bits)
    // We already caught the first K edge in ISR
    
    uint32_t bitTime = 0;
    uint32_t lastLevel = (*gpio_in) & dpMask;
    uint32_t startCycle = esp_cpu_get_cycle_count();
    
    // Skip remaining SYNC bits - look for stable K followed by data
    // SYNC ends with KK (D+ high, high), then PID starts
    int syncBits = 0;
    while (syncBits < 16) {
        uint32_t now = esp_cpu_get_cycle_count();
        if ((now - startCycle) > CYCLES_PER_BIT) {
            startCycle = now;
            syncBits++;
            uint32_t level = (*gpio_in) & dpMask;
            
            // Detect KK pattern (two consecutive D+ high)
            if (level && lastLevel) {
                // End of SYNC found
                break;
            }
            lastLevel = level;
        }
        
        // Timeout after ~20 bits
        if (syncBits > 16) return 0;
    }
    
    // Now decode NRZI data
    uint8_t byte = 0;
    int bitPos = 0;
    int byteCount = 0;
    int onesCount = 0;
    uint32_t prevLevel = (*gpio_in) & dpMask;
    
    startCycle = esp_cpu_get_cycle_count();
    
    // Sample at center of each bit
    while (byteCount < maxLen) {
        // Wait for center of next bit
        while ((esp_cpu_get_cycle_count() - startCycle) < CYCLES_PER_BIT) {}
        startCycle = esp_cpu_get_cycle_count();
        
        uint32_t level = (*gpio_in) & dpMask;
        
        // Check for SE0 (both lines low = EOP)
        uint32_t dm = (*gpio_in) & _dmMask;
        if (level == 0 && dm == 0) {
            // End of packet
            break;
        }
        
        // NRZI decode: same level = 1, different = 0
        uint8_t dataBit;
        if (level == prevLevel) {
            dataBit = 1;
            onesCount++;
            
            // Bit stuffing: after 6 ones, next is stuffed 0
            if (onesCount == 6) {
                // Skip stuffed bit
                onesCount = 0;
                while ((esp_cpu_get_cycle_count() - startCycle) < CYCLES_PER_BIT) {}
                startCycle = esp_cpu_get_cycle_count();
                prevLevel = (*gpio_in) & dpMask;
                continue;
            }
        } else {
            dataBit = 0;
            onesCount = 0;
        }
        
        prevLevel = level;
        
        // Assemble byte (LSB first)
        byte |= (dataBit << bitPos);
        bitPos++;
        
        if (bitPos == 8) {
            buffer[byteCount++] = byte;
            byte = 0;
            bitPos = 0;
        }
    }
    
    return byteCount;
}

// Send packet with SYNC, data, and EOP
void IRAM_ATTR SoftUSBKeyboard_GPIO::sendPacket(const uint8_t* data, int len) {
    uint32_t dpMask = _dpMask;
    uint32_t dmMask = _dmMask;
    
    // Switch to output mode
    gpio_set_direction((gpio_num_t)_pinDp, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)_pinDm, GPIO_MODE_OUTPUT);
    
    // Start from J state (D+ low, D- high for Low-Speed)
    GPIO_OUT_CLR = dpMask;
    GPIO_OUT_SET = dmMask;
    delayBit();
    
    // Send SYNC: KJKJKJKK (8 bits)
    // K = D+ high, D- low
    // J = D+ low, D- high
    static const uint8_t syncPattern[] = {1,0,1,0,1,0,1,1}; // K,J,K,J,K,J,K,K
    for (int i = 0; i < 8; i++) {
        if (syncPattern[i]) {
            GPIO_OUT_SET = dpMask;
            GPIO_OUT_CLR = dmMask;
        } else {
            GPIO_OUT_CLR = dpMask;
            GPIO_OUT_SET = dmMask;
        }
        delayBit();
    }
    
    // Current state after SYNC: K (D+ high)
    uint8_t currentLevel = 1;
    int onesCount = 0;
    
    // Send data with NRZI encoding and bit stuffing
    for (int byteIdx = 0; byteIdx < len; byteIdx++) {
        uint8_t byte = data[byteIdx];
        
        for (int bitIdx = 0; bitIdx < 8; bitIdx++) {
            uint8_t dataBit = (byte >> bitIdx) & 1;
            
            if (dataBit == 1) {
                // Stay at current level
                onesCount++;
            } else {
                // Toggle level
                currentLevel = !currentLevel;
                onesCount = 0;
            }
            
            if (currentLevel) {
                GPIO_OUT_SET = dpMask;
                GPIO_OUT_CLR = dmMask;
            } else {
                GPIO_OUT_CLR = dpMask;
                GPIO_OUT_SET = dmMask;
            }
            delayBit();
            
            // Bit stuffing after 6 ones
            if (onesCount == 6) {
                currentLevel = !currentLevel;
                if (currentLevel) {
                    GPIO_OUT_SET = dpMask;
                    GPIO_OUT_CLR = dmMask;
                } else {
                    GPIO_OUT_CLR = dpMask;
                    GPIO_OUT_SET = dmMask;
                }
                delayBit();
                onesCount = 0;
            }
        }
    }
    
    // Send EOP: SE0 for 2 bit times, then J for 1 bit
    GPIO_OUT_CLR = dpMask | dmMask;  // SE0
    delayBit();
    delayBit();
    
    GPIO_OUT_CLR = dpMask;  // J state
    GPIO_OUT_SET = dmMask;
    delayBit();
    
    // Return to input mode
    gpio_set_direction((gpio_num_t)_pinDp, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)_pinDm, GPIO_MODE_INPUT);
}

void IRAM_ATTR SoftUSBKeyboard_GPIO::sendHandshake(uint8_t pid) {
    uint8_t packet[1] = {pid};
    sendPacket(packet, 1);
}

void IRAM_ATTR SoftUSBKeyboard_GPIO::sendDataPacket(uint8_t pid, const uint8_t* data, int len) {
    uint8_t packet[64];
    packet[0] = pid;
    if (data && len > 0) {
        memcpy(&packet[1], data, len);
    }
    
    // Calculate CRC16
    uint16_t crc = crc16(data, len);
    packet[1 + len] = crc & 0xFF;
    packet[2 + len] = (crc >> 8) & 0xFF;
    
    sendPacket(packet, len + 3);
}

void SoftUSBKeyboard_GPIO::handleSetupPacket(const uint8_t* setup) {
    uint8_t bmRequestType = setup[0];
    uint8_t bRequest = setup[1];
    uint16_t wValue = setup[2] | (setup[3] << 8);
    uint16_t wIndex = setup[4] | (setup[5] << 8);
    uint16_t wLength = setup[6] | (setup[7] << 8);
    
    _ctrlState = USB_STATE_IDLE;
    _txLen = 0;
    _txOffset = 0;
    _dataPID = true;  // First data packet is DATA1
    
    if (bmRequestType == 0x80) {
        // Standard device request (IN)
        if (bRequest == 0x06) {
            // GET_DESCRIPTOR
            handleGetDescriptor(wValue >> 8, wValue & 0xFF, wLength);
        }
    } else if (bmRequestType == 0x00) {
        // Standard device request (OUT)
        if (bRequest == 0x05) {
            // SET_ADDRESS
            _pendingAddress = wValue & 0x7F;
            // Send zero-length DATA1 for status
            _txLen = 0;
            _ctrlState = USB_STATE_DATA_IN;
        } else if (bRequest == 0x09) {
            // SET_CONFIGURATION
            _configured = (wValue != 0);
            _txLen = 0;
            _ctrlState = USB_STATE_DATA_IN;
        }
    } else if (bmRequestType == 0x81) {
        // Interface request (IN)
        if (bRequest == 0x06) {
            // GET_DESCRIPTOR (HID)
            handleGetDescriptor(wValue >> 8, wValue & 0xFF, wLength);
        }
    } else if (bmRequestType == 0x21) {
        // Class request (OUT)
        if (bRequest == 0x0A) {
            // SET_IDLE
            _txLen = 0;
            _ctrlState = USB_STATE_DATA_IN;
        }
    }
}

void SoftUSBKeyboard_GPIO::handleGetDescriptor(uint8_t descType, uint8_t descIndex, uint16_t wLength) {
    const uint8_t* desc = nullptr;
    int descLen = 0;
    
    switch (descType) {
        case USB_DESC_DEVICE:
            desc = deviceDescriptor;
            descLen = sizeof(deviceDescriptor);
            break;
            
        case USB_DESC_CONFIGURATION:
            desc = configDescriptor;
            descLen = sizeof(configDescriptor);
            break;
            
        case USB_DESC_STRING:
            switch (descIndex) {
                case 0: desc = stringDesc0; descLen = sizeof(stringDesc0); break;
                case 1: desc = stringDesc1; descLen = sizeof(stringDesc1); break;
                case 2: desc = stringDesc2; descLen = sizeof(stringDesc2); break;
            }
            break;
            
        case USB_DESC_HID_REPORT:
            desc = hidReportDescriptor;
            descLen = sizeof(hidReportDescriptor);
            break;
    }
    
    if (desc && descLen > 0) {
        if (wLength < descLen) descLen = wLength;
        memcpy(_txBuffer, desc, descLen);
        _txLen = descLen;
        _txOffset = 0;
        _ctrlState = USB_STATE_DATA_IN;
    }
}

void SoftUSBKeyboard_GPIO::task() {
    // Poll for USB activity instead of using interrupts
    // (GPIO18/19 are USB pins and may not support standard GPIO interrupts)
    
    uint32_t dpMask = _dpMask;
    volatile uint32_t* gpio_in = &GPIO.in.val;
    
    // Check for activity - Low-Speed idle is D+=0, D-=1
    // Packet starts with SYNC which has D+ going high
    if ((*gpio_in) & dpMask) {
        _isrCount = _isrCount + 1;
        handleUSB();
    }
    
    // Print status occasionally
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 2000) {
        lastPrint = millis();
        Serial.printf("USB_GPIO: addr=%d cfg=%d poll=%lu lastLen=%d\n", 
                      _address, _configured ? 1 : 0, _isrCount, _lastRxLen);
    }
}

void SoftUSBKeyboard_GPIO::pressKey(uint8_t key) {
    for (int i = 2; i < 8; i++) {
        if (_keyReport[i] == 0) {
            _keyReport[i] = key;
            _reportChanged = true;
            break;
        }
    }
}

void SoftUSBKeyboard_GPIO::releaseKey(uint8_t key) {
    for (int i = 2; i < 8; i++) {
        if (_keyReport[i] == key) {
            _keyReport[i] = 0;
            _reportChanged = true;
            break;
        }
    }
}

void SoftUSBKeyboard_GPIO::releaseAll() {
    memset(_keyReport, 0, sizeof(_keyReport));
    _reportChanged = true;
}

bool SoftUSBKeyboard_GPIO::isReady() {
    return _configured;
}

inline void IRAM_ATTR SoftUSBKeyboard_GPIO::delayBit() {
    uint32_t start = esp_cpu_get_cycle_count();
    while ((esp_cpu_get_cycle_count() - start) < CYCLES_PER_BIT) {}
}

inline void IRAM_ATTR SoftUSBKeyboard_GPIO::delayHalfBit() {
    uint32_t start = esp_cpu_get_cycle_count();
    while ((esp_cpu_get_cycle_count() - start) < CYCLES_HALF_BIT) {}
}

uint8_t SoftUSBKeyboard_GPIO::crc5(uint16_t data) {
    uint8_t crc = 0x1F;
    for (int i = 0; i < 11; i++) {
        uint8_t bit = (data >> i) & 1;
        if ((crc & 1) ^ bit) crc = (crc >> 1) ^ 0x14;
        else crc >>= 1;
    }
    return crc ^ 0x1F;
}

uint16_t SoftUSBKeyboard_GPIO::crc16(const uint8_t* data, int len) {
    uint16_t crc = 0xFFFF;
    if (data) {
        for (int i = 0; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 1) crc = (crc >> 1) ^ 0xA001;
                else crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFF;
}
