/**
 * SoftUSB - Software Low-Speed USB HID Keyboard Implementation
 * 
 * ESP32-C3 @ 160MHz = 6.25ns per cycle
 * USB Low-Speed = 1.5Mbps = 666.67ns per bit = ~107 cycles per bit
 */

#include "SoftUSB.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "soc/usb_serial_jtag_struct.h"
#include "soc/usb_serial_jtag_reg.h"
#include "esp_cpu.h"

// Disable interrupts for critical timing sections
#define ENTER_CRITICAL() portDISABLE_INTERRUPTS()
#define EXIT_CRITICAL()  portENABLE_INTERRUPTS()

// USB Device Descriptor
static const uint8_t deviceDescriptor[] = {
    18,                 // bLength
    USB_DESC_DEVICE,    // bDescriptorType
    0x10, 0x01,         // bcdUSB (USB 1.1)
    0x00,               // bDeviceClass (defined at interface level)
    0x00,               // bDeviceSubClass
    0x00,               // bDeviceProtocol
    8,                  // bMaxPacketSize0
    0x34, 0x12,         // idVendor (0x1234 - testing)
    0xCD, 0xAB,         // idProduct (0xABCD)
    0x00, 0x01,         // bcdDevice (1.00)
    1,                  // iManufacturer
    2,                  // iProduct
    0,                  // iSerialNumber
    1                   // bNumConfigurations
};

// HID Report Descriptor (standard keyboard)
static const uint8_t hidReportDescriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0xE0,        //   Usage Minimum (224)
    0x29, 0xE7,        //   Usage Maximum (231)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data, Variable, Absolute) - Modifier byte
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x01,        //   Input (Constant) - Reserved byte
    0x95, 0x06,        //   Report Count (6)
    0x75, 0x08,        //   Report Size (8)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (101)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0x00,        //   Usage Minimum (0)
    0x29, 0x65,        //   Usage Maximum (101)
    0x81, 0x00,        //   Input (Data, Array) - Key array
    0xC0               // End Collection
};

// Configuration Descriptor (includes interface, HID, endpoint)
static const uint8_t configDescriptor[] = {
    // Configuration Descriptor
    9,                      // bLength
    USB_DESC_CONFIGURATION, // bDescriptorType
    34, 0,                  // wTotalLength (9+9+9+7=34)
    1,                      // bNumInterfaces
    1,                      // bConfigurationValue
    0,                      // iConfiguration
    0xA0,                   // bmAttributes (bus powered, remote wakeup)
    50,                     // bMaxPower (100mA)
    
    // Interface Descriptor
    9,                      // bLength
    USB_DESC_INTERFACE,     // bDescriptorType
    0,                      // bInterfaceNumber
    0,                      // bAlternateSetting
    1,                      // bNumEndpoints
    0x03,                   // bInterfaceClass (HID)
    0x01,                   // bInterfaceSubClass (Boot)
    0x01,                   // bInterfaceProtocol (Keyboard)
    0,                      // iInterface
    
    // HID Descriptor
    9,                      // bLength
    USB_DESC_HID,           // bDescriptorType
    0x11, 0x01,             // bcdHID (1.11)
    0,                      // bCountryCode
    1,                      // bNumDescriptors
    USB_DESC_HID_REPORT,    // bDescriptorType
    sizeof(hidReportDescriptor), 0, // wDescriptorLength
    
    // Endpoint Descriptor (EP1 IN, Interrupt)
    7,                      // bLength
    USB_DESC_ENDPOINT,      // bDescriptorType
    0x81,                   // bEndpointAddress (EP1 IN)
    0x03,                   // bmAttributes (Interrupt)
    8, 0,                   // wMaxPacketSize
    10                      // bInterval (10ms)
};

// String Descriptors
static const uint8_t stringDescLang[] = {4, USB_DESC_STRING, 0x09, 0x04}; // English
static const uint8_t stringDescMfg[] = {14, USB_DESC_STRING, 'S',0,'o',0,'f',0,'t',0,'U',0,'S',0,'B',0};
static const uint8_t stringDescProd[] = {20, USB_DESC_STRING, 'K',0,'e',0,'y',0,'b',0,'o',0,'a',0,'r',0,'d',0};

// Constructor
SoftUSBKeyboard::SoftUSBKeyboard(uint8_t pinDp, uint8_t pinDm) 
    : _pinDp(pinDp), _pinDm(pinDm), _configured(false), _address(0), 
      _dataToggle(false), _reportChanged(false) {
    memset(_keyReport, 0, sizeof(_keyReport));
}

bool SoftUSBKeyboard::begin() {
    // Disable ESP32-C3's built-in USB Serial/JTAG to free GPIO18/19
    USB_SERIAL_JTAG.conf0.usb_pad_enable = 0;
    USB_SERIAL_JTAG.conf0.dp_pullup = 0;  // Disable D+ pullup from internal USB
    
    // First, configure D+ (must have NO pull-up for Low-Speed)
    gpio_config_t dp_conf = {};
    dp_conf.intr_type = GPIO_INTR_DISABLE;
    dp_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    dp_conf.pin_bit_mask = (1ULL << _pinDp);
    dp_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    dp_conf.pull_up_en = GPIO_PULLUP_DISABLE; // NO pull-up on D+!
    gpio_config(&dp_conf);
    gpio_set_pull_mode((gpio_num_t)_pinDp, GPIO_FLOATING);
    
    // Configure D- with pull-up (signals Low-Speed device)
    gpio_config_t dm_conf = {};
    dm_conf.intr_type = GPIO_INTR_DISABLE;
    dm_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    dm_conf.pin_bit_mask = (1ULL << _pinDm);
    dm_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    dm_conf.pull_up_en = GPIO_PULLUP_ENABLE; // Pull-up on D- for Low-Speed
    gpio_config(&dm_conf);
    gpio_set_pull_mode((gpio_num_t)_pinDm, GPIO_PULLUP_ONLY);
    
    Serial.printf("SoftUSB: D+ on GPIO%d (no pullup), D- on GPIO%d (pullup)\n", _pinDp, _pinDm);
    Serial.println("SoftUSB: Internal USB disabled, using GPIO mode");
    
    // Start in idle state (J state for LS: D+ low, D- high)
    // With internal pull-up on D-, the line will be pulled high
    setInput();
    
    // Wait for lines to stabilize with pull-ups
    delayMicroseconds(100);
    
    // Signal device attachment with SE0 then J
    ENTER_CRITICAL();
    setOutput();
    sendSE0(10); // 10us SE0
    sendJ();     // Then J state
    setInput();
    EXIT_CRITICAL();
    
    Serial.println("SoftUSB: Device attached, ready for enumeration");
    Serial.printf("Initial state: D+=%d D-=%d\n", getDP(), getDM());
    
    return true;
}

// Inline GPIO functions for speed
inline void SoftUSBKeyboard::setDP(bool high) {
    if (high) {
        GPIO.out_w1ts.val = (1 << _pinDp);
    } else {
        GPIO.out_w1tc.val = (1 << _pinDp);
    }
}

inline void SoftUSBKeyboard::setDM(bool high) {
    if (high) {
        GPIO.out_w1ts.val = (1 << _pinDm);
    } else {
        GPIO.out_w1tc.val = (1 << _pinDm);
    }
}

inline bool SoftUSBKeyboard::getDP() {
    return (GPIO.in.val >> _pinDp) & 1;
}

inline bool SoftUSBKeyboard::getDM() {
    return (GPIO.in.val >> _pinDm) & 1;
}

inline void SoftUSBKeyboard::setOutput() {
    gpio_set_direction((gpio_num_t)_pinDp, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)_pinDm, GPIO_MODE_OUTPUT);
}

inline void SoftUSBKeyboard::setInput() {
    gpio_set_direction((gpio_num_t)_pinDp, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)_pinDm, GPIO_MODE_INPUT);
}

// Delay for one bit period (666.67ns at 1.5Mbps)
// At 160MHz, ~107 cycles per bit
// Using inline NOP for minimal overhead
void SoftUSBKeyboard::delayBit() {
    // 107 cycles = ~17 iterations of 6-cycle loop + overhead
    // Tuned experimentally
    __asm__ volatile(
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"  // 10
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"  // 20
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"  // 30
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"  // 40
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"  // 50
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"  // 60
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"  // 70
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"  // 80
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"  // 90
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\n"                 // 97 (+ ~10 overhead)
    );
}

void SoftUSBKeyboard::delayHalfBit() {
    __asm__ volatile(
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"  // 10
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"  // 20
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"  // 30
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"  // 40
        "nop\nnop\nnop"                                       // 43 (+ ~10 overhead)
    );
}

// Send SE0 (Single-Ended Zero: both D+ and D- low)
void SoftUSBKeyboard::sendSE0(uint16_t durationUs) {
    setOutput();
    setDP(false);
    setDM(false);
    delayMicroseconds(durationUs);
}

// Send J state (Idle for Low-Speed: D+ low via pullup, D- high)
// For LS device driving: D+ = 0, D- = 1
void SoftUSBKeyboard::sendJ() {
    setDP(false);
    setDM(true);
}

// Send K state (opposite of J)
// For LS device: D+ = 1, D- = 0
void SoftUSBKeyboard::sendK() {
    setDP(true);
    setDM(false);
}

// Send a single bit using NRZI encoding
// NRZI: 0 = transition, 1 = no transition
void SoftUSBKeyboard::sendBit(uint8_t bit) {
    static bool currentState = false; // false = J, true = K
    
    if (bit == 0) {
        // Transition
        currentState = !currentState;
    }
    // else: no transition (bit = 1)
    
    if (currentState) {
        sendK();
    } else {
        sendJ();
    }
    delayBit();
}

// Send a byte LSB first with bit stuffing
void SoftUSBKeyboard::sendByte(uint8_t byte) {
    static uint8_t onesCount = 0;
    
    for (int i = 0; i < 8; i++) {
        uint8_t bit = (byte >> i) & 1;
        sendBit(bit);
        
        // Bit stuffing: after 6 consecutive 1s, insert a 0
        if (bit == 1) {
            onesCount++;
            if (onesCount == 6) {
                sendBit(0); // Stuff bit
                onesCount = 0;
            }
        } else {
            onesCount = 0;
        }
    }
}

// Calculate CRC5 for token packets (11 bits of data)
uint8_t SoftUSBKeyboard::crc5(uint16_t data) {
    uint8_t crc = 0x1F; // Initial value
    for (int i = 0; i < 11; i++) {
        uint8_t bit = (data >> i) & 1;
        uint8_t xorBit = (crc & 1) ^ bit;
        crc >>= 1;
        if (xorBit) {
            crc ^= 0x14; // Polynomial: x^5 + x^2 + 1
        }
    }
    return crc ^ 0x1F; // Invert
}

// Calculate CRC16 for data packets
uint16_t SoftUSBKeyboard::crc16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (int j = 0; j < 8; j++) {
            uint8_t bit = (byte >> j) & 1;
            uint8_t xorBit = (crc & 1) ^ bit;
            crc >>= 1;
            if (xorBit) {
                crc ^= 0xA001; // Polynomial: x^16 + x^15 + x^2 + 1
            }
        }
    }
    return crc ^ 0xFFFF;
}

// Send a complete USB packet
void SoftUSBKeyboard::sendPacket(const uint8_t* data, uint8_t len) {
    ENTER_CRITICAL();
    setOutput();
    
    // SYNC pattern: KJKJKJKK (8 bits: 00000001 in NRZI)
    sendK(); delayBit();
    sendJ(); delayBit();
    sendK(); delayBit();
    sendJ(); delayBit();
    sendK(); delayBit();
    sendJ(); delayBit();
    sendK(); delayBit();
    sendK(); delayBit();
    
    // Data
    for (uint8_t i = 0; i < len; i++) {
        sendByte(data[i]);
    }
    
    // EOP: SE0 for 2 bit times, then J
    sendSE0(2);
    sendJ();
    delayBit();
    
    setInput();
    EXIT_CRITICAL();
}

// Send handshake packet (ACK, NAK, STALL)
void SoftUSBKeyboard::sendHandshake(uint8_t pid) {
    uint8_t packet[1] = {pid};
    sendPacket(packet, 1);
}

// Send data packet with CRC
void SoftUSBKeyboard::sendDataPacket(uint8_t pid, const uint8_t* data, uint8_t len) {
    uint8_t packet[64];
    packet[0] = pid;
    memcpy(&packet[1], data, len);
    
    // Calculate and append CRC16
    uint16_t crc = crc16(data, len);
    packet[1 + len] = crc & 0xFF;
    packet[2 + len] = (crc >> 8) & 0xFF;
    
    sendPacket(packet, len + 3);
}

// Edge buffer - records timestamp and state at each transition
// Format: lower 8 bits = cycle count / 8, upper 8 bits = D+/D- state
#define EDGE_BUF_SIZE 128
static uint16_t edgeBuf[EDGE_BUF_SIZE];
static volatile uint8_t edgeCount;

// Get cycle count divided by 8 (for fitting in 8 bits with enough resolution)
static inline uint8_t getCycleCount8() {
    return (uint8_t)(esp_cpu_get_cycle_count() >> 3);
}

// Wait for and receive a packet using edge-triggered recording
// This method is inspired by sdima1357's esp32_usb_soft_host
bool SoftUSBKeyboard::waitForPacket(uint8_t* buffer, uint8_t* len, uint32_t timeoutUs) {
    uint32_t dpMask = (1 << _pinDp);
    uint32_t dmMask = (1 << _pinDm);
    uint32_t bothMask = dpMask | dmMask;
    volatile uint32_t* inReg = &GPIO.in.val;
    
    // Wait for K state (D+ high) - this is non-critical so don't block too long
    uint32_t start = micros();
    while (!(*inReg & dpMask)) {
        if (micros() - start > timeoutUs) return false;
    }
    
    // Now capture edges with timestamps
    ENTER_CRITICAL();
    
    uint32_t val = (*inReg & bothMask);
    uint32_t nval;
    uint8_t recIdx = 0;
    int timeout = 800; // ~5us at 160MHz with this loop
    
    edgeBuf[recIdx++] = getCycleCount8() | ((val >> (_pinDp > _pinDm ? _pinDm : _pinDp)) << 8);
    
    while (recIdx < EDGE_BUF_SIZE && timeout > 0) {
        nval = (*inReg & bothMask);
        if (nval != val) {
            edgeBuf[recIdx++] = getCycleCount8() | ((nval >> (_pinDp > _pinDm ? _pinDm : _pinDp)) << 8);
            val = nval;
            timeout = 800;
        } else {
            timeout--;
        }
    }
    
    EXIT_CRITICAL();
    edgeCount = recIdx;
    
    if (recIdx < 10) return false; // Not enough edges for a packet
    
    // Decode the packet from edge transitions
    // Calculate bit time from SYNC pattern (KJKJKJKK = alternating)
    // First few transitions should be ~1 bit apart
    
    int bitTime = 0;
    int transCount = 0;
    for (int i = 1; i < recIdx && i < 8; i++) {
        int delta = (edgeBuf[i] & 0xFF) - (edgeBuf[i-1] & 0xFF);
        if (delta < 0) delta += 256; // Handle wraparound
        if (delta > 5 && delta < 30) { // Reasonable bit time range (8-15 cycles/8 = 1-2 bit times)
            bitTime += delta;
            transCount++;
        }
    }
    if (transCount > 0) {
        bitTime = bitTime / transCount;
    } else {
        bitTime = 13; // ~107 cycles / 8 = ~13
    }
    
    // Decode NRZI from edges
    // Each edge represents a '0' bit, no edge for 1 bit time = '1' bit
    *len = 0;
    uint8_t currentByte = 0;
    int bitPos = 0;
    int onesCount = 0;
    bool inSync = true;
    int syncBits = 0;
    
    for (int i = 1; i < recIdx; i++) {
        int delta = (edgeBuf[i] & 0xFF) - (edgeBuf[i-1] & 0xFF);
        if (delta < 0) delta += 256;
        
        // How many bit periods in this delta?
        int bits = (delta + bitTime/2) / bitTime;
        if (bits < 1) bits = 1;
        if (bits > 8) bits = 8;
        
        // bits-1 ones followed by one zero (edge = 0 in NRZI)
        for (int b = 0; b < bits - 1; b++) {
            if (inSync) {
                syncBits++;
                if (syncBits >= 7) { // End of SYNC (last bit is 0 at transition)
                    inSync = false;
                }
                continue;
            }
            
            // Handle bit stuffing
            onesCount++;
            if (onesCount == 6) {
                onesCount = 0;
                continue; // Skip stuff bit
            }
            
            currentByte |= (1 << bitPos);
            bitPos++;
            if (bitPos == 8) {
                buffer[(*len)++] = currentByte;
                currentByte = 0;
                bitPos = 0;
                if (*len >= 16) break;
            }
        }
        
        if (*len >= 16) break;
        
        // The transition itself represents a 0 bit
        if (inSync) {
            syncBits++;
            continue;
        }
        
        onesCount = 0;
        // 0 bit - don't set the bit (already 0)
        bitPos++;
        if (bitPos == 8) {
            buffer[(*len)++] = currentByte;
            currentByte = 0;
            bitPos = 0;
            if (*len >= 16) break;
        }
        
        // Check for SE0 (end of packet)
        uint8_t state = edgeBuf[i] >> 8;
        if (state == 0) break; // SE0
    }
    
    // Handle any remaining bits
    if (bitPos > 0 && *len < 16) {
        buffer[(*len)++] = currentByte;
    }
    
    return *len > 0;
}

// Main task - must be called frequently
void SoftUSBKeyboard::task() {
    static uint32_t lastDebug = 0;
    static uint32_t packetCount = 0;
    static uint32_t edgeCount = 0;
    static bool lastDp = false;
    
    // Simple edge detection to verify we see activity
    bool dp = getDP();
    if (dp != lastDp) {
        edgeCount++;
        lastDp = dp;
    }
    
    // Debug: show line states every second
    if (millis() - lastDebug > 1000) {
        lastDebug = millis();
        Serial.printf("USB: D+=%d D-=%d, edges=%lu, pkts=%lu\n", 
                      getDP(), getDM(), edgeCount, packetCount);
        edgeCount = 0; // Reset edge counter
    }
    
    uint8_t buffer[64];
    uint8_t len;
    
    // Check for incoming packet (5ms timeout to catch host retries)
    if (waitForPacket(buffer, &len, 5000)) {
        packetCount++;
        if (len < 1) return;
        
        uint8_t pid = buffer[0];
        // Print all received bytes for debug
        Serial.printf("RX len=%d: ", len);
        for (int i = 0; i < len && i < 16; i++) {
            Serial.printf("%02X ", buffer[i]);
        }
        Serial.println();
        
        switch (pid) {
            case USB_PID_SETUP:
                Serial.println("-> SETUP token");
                // Next packet should be DATA0 with setup data
                if (waitForPacket(buffer, &len, 5000) && buffer[0] == USB_PID_DATA0) {
                    Serial.printf("-> DATA0 len=%d\n", len);
                    sendHandshake(USB_PID_ACK);
                    handleSetupPacket(&buffer[1]);
                } else {
                    Serial.println("-> No DATA0 received");
                }
                break;
                
            case USB_PID_IN:
                // Host wants data from us
                if (_configured && _reportChanged) {
                    sendDataPacket(_dataToggle ? USB_PID_DATA1 : USB_PID_DATA0, 
                                   _keyReport, 8);
                    _dataToggle = !_dataToggle;
                    _reportChanged = false;
                } else {
                    sendHandshake(USB_PID_NAK);
                }
                break;
                
            case USB_PID_OUT:
                // Host sending data to us
                if (waitForPacket(buffer, &len, 1000)) {
                    sendHandshake(USB_PID_ACK);
                }
                break;
                
            default:
                Serial.printf("-> Unknown PID: 0x%02X\n", pid);
                break;
        }
    }
}

// Handle SETUP packet
void SoftUSBKeyboard::handleSetupPacket(const uint8_t* setup) {
    uint8_t bmRequestType = setup[0];
    uint8_t bRequest = setup[1];
    uint16_t wValue = setup[2] | (setup[3] << 8);
    uint16_t wIndex = setup[4] | (setup[5] << 8);
    uint16_t wLength = setup[6] | (setup[7] << 8);
    
    uint8_t type = (bmRequestType >> 5) & 0x03;
    
    if (type == 0) { // Standard request
        switch (bRequest) {
            case USB_REQ_GET_DESCRIPTOR:
                handleGetDescriptor((wValue >> 8), (wValue & 0xFF), wLength);
                break;
                
            case USB_REQ_SET_ADDRESS:
                _address = wValue & 0x7F;
                // Send zero-length DATA packet
                sendDataPacket(USB_PID_DATA1, NULL, 0);
                Serial.printf("SoftUSB: Address set to %d\n", _address);
                break;
                
            case USB_REQ_SET_CONFIGURATION:
                _configured = true;
                sendDataPacket(USB_PID_DATA1, NULL, 0);
                Serial.println("SoftUSB: Device configured");
                break;
                
            case USB_REQ_GET_CONFIGURATION:
                {
                    uint8_t config = _configured ? 1 : 0;
                    sendDataPacket(USB_PID_DATA1, &config, 1);
                }
                break;
                
            default:
                sendHandshake(USB_PID_STALL);
                break;
        }
    } else if (type == 1) { // Class request (HID)
        handleHIDRequest(setup);
    } else {
        sendHandshake(USB_PID_STALL);
    }
}

// Handle GET_DESCRIPTOR request
void SoftUSBKeyboard::handleGetDescriptor(uint8_t type, uint8_t index, uint16_t length) {
    const uint8_t* desc = NULL;
    uint8_t descLen = 0;
    
    switch (type) {
        case USB_DESC_DEVICE:
            desc = deviceDescriptor;
            descLen = sizeof(deviceDescriptor);
            break;
            
        case USB_DESC_CONFIGURATION:
            desc = configDescriptor;
            descLen = sizeof(configDescriptor);
            break;
            
        case USB_DESC_STRING:
            switch (index) {
                case 0: desc = stringDescLang; descLen = sizeof(stringDescLang); break;
                case 1: desc = stringDescMfg; descLen = sizeof(stringDescMfg); break;
                case 2: desc = stringDescProd; descLen = sizeof(stringDescProd); break;
            }
            break;
            
        case USB_DESC_HID_REPORT:
            desc = hidReportDescriptor;
            descLen = sizeof(hidReportDescriptor);
            break;
    }
    
    if (desc) {
        uint8_t sendLen = min((uint8_t)length, descLen);
        // Send in chunks of 8 bytes (max packet size for LS)
        uint8_t offset = 0;
        while (offset < sendLen) {
            uint8_t chunkLen = min((uint8_t)8, (uint8_t)(sendLen - offset));
            sendDataPacket(_dataToggle ? USB_PID_DATA1 : USB_PID_DATA0,
                          desc + offset, chunkLen);
            _dataToggle = !_dataToggle;
            offset += chunkLen;
            
            // Wait for ACK
            uint8_t buffer[8];
            uint8_t len;
            waitForPacket(buffer, &len, 1000);
        }
    } else {
        sendHandshake(USB_PID_STALL);
    }
}

// Handle HID class requests
void SoftUSBKeyboard::handleHIDRequest(const uint8_t* setup) {
    uint8_t bRequest = setup[1];
    
    switch (bRequest) {
        case HID_REQ_GET_REPORT:
            sendDataPacket(USB_PID_DATA1, _keyReport, 8);
            break;
            
        case HID_REQ_SET_IDLE:
            sendDataPacket(USB_PID_DATA1, NULL, 0);
            break;
            
        case HID_REQ_GET_PROTOCOL:
            {
                uint8_t protocol = 1; // Report protocol
                sendDataPacket(USB_PID_DATA1, &protocol, 1);
            }
            break;
            
        case HID_REQ_SET_PROTOCOL:
            sendDataPacket(USB_PID_DATA1, NULL, 0);
            break;
            
        default:
            sendHandshake(USB_PID_STALL);
            break;
    }
}

// Press a key
void SoftUSBKeyboard::pressKey(uint8_t keycode) {
    // Find empty slot in key array
    for (int i = 2; i < 8; i++) {
        if (_keyReport[i] == 0) {
            _keyReport[i] = keycode;
            _reportChanged = true;
            break;
        }
    }
}

// Release a key
void SoftUSBKeyboard::releaseKey(uint8_t keycode) {
    for (int i = 2; i < 8; i++) {
        if (_keyReport[i] == keycode) {
            _keyReport[i] = 0;
            _reportChanged = true;
            break;
        }
    }
}

// Release all keys
void SoftUSBKeyboard::releaseAll() {
    memset(_keyReport, 0, sizeof(_keyReport));
    _reportChanged = true;
}
