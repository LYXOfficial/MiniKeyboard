/**
 * SoftUSB_RMT - Software Low-Speed USB using RMT peripheral
 * 
 * Strategy: Use RMT RX for precise capture, bit-bang for TX
 * RMT RX is critical for reliable packet reception
 * TX timing is less critical (device responds, host is tolerant)
 */

#include "SoftUSB_RMT.h"
#include "soc/gpio_struct.h"
#include "soc/usb_serial_jtag_struct.h"
#include "esp_cpu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// RMT RX callback data
static volatile bool rxDone = false;
static volatile size_t rxSymbolCount = 0;
static rmt_symbol_word_t rxSymbolBuffer[RMT_RX_BUF_SIZE];
static volatile uint32_t rxDoneTime = 0;

// RMT RX done callback
static bool IRAM_ATTR rmt_rx_done_callback(rmt_channel_handle_t channel,
                                            const rmt_rx_done_event_data_t *edata,
                                            void *user_data) {
    rxDoneTime = esp_cpu_get_cycle_count();
    rxSymbolCount = edata->num_symbols;
    if (rxSymbolCount > RMT_RX_BUF_SIZE) rxSymbolCount = RMT_RX_BUF_SIZE;
    memcpy(rxSymbolBuffer, edata->received_symbols, rxSymbolCount * sizeof(rmt_symbol_word_t));
    rxDone = true;
    return false;
}

// USB Device Descriptor
static const uint8_t deviceDescriptor[] = {
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

// HID Report Descriptor (Keyboard)
static const uint8_t hidReportDescriptor[] = {
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
    0x81, 0x02,  //   Input (Data, Variable, Absolute) - Modifier byte
    0x95, 0x01,  //   Report Count (1)
    0x75, 0x08,  //   Report Size (8)
    0x81, 0x01,  //   Input (Constant) - Reserved byte
    0x95, 0x06,  //   Report Count (6)
    0x75, 0x08,  //   Report Size (8)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x65,  //   Logical Maximum (101)
    0x05, 0x07,  //   Usage Page (Key Codes)
    0x19, 0x00,  //   Usage Minimum (0)
    0x29, 0x65,  //   Usage Maximum (101)
    0x81, 0x00,  //   Input (Data, Array) - Key arrays (6 bytes)
    0xC0         // End Collection
};

// Configuration Descriptor
static const uint8_t configDescriptor[] = {
    // Configuration Descriptor
    9,                      // bLength
    USB_DESC_CONFIGURATION, // bDescriptorType
    34, 0,                  // wTotalLength
    1,                      // bNumInterfaces
    1,                      // bConfigurationValue
    0,                      // iConfiguration
    0x80,                   // bmAttributes (Bus Powered)
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
    0x00,                   // bCountryCode
    1,                      // bNumDescriptors
    USB_DESC_HID_REPORT,    // bDescriptorType
    sizeof(hidReportDescriptor), 0, // wDescriptorLength
    
    // Endpoint Descriptor
    7,                      // bLength
    USB_DESC_ENDPOINT,      // bDescriptorType
    0x81,                   // bEndpointAddress (IN 1)
    0x03,                   // bmAttributes (Interrupt)
    8, 0,                   // wMaxPacketSize
    10                      // bInterval
};

static const uint8_t stringDescLang[] = {4, USB_DESC_STRING, 0x09, 0x04};
static const uint8_t stringDescMfg[] = {16, USB_DESC_STRING, 'S',0,'o',0,'f',0,'t',0,'U',0,'S',0,'B',0};
static const uint8_t stringDescProd[] = {18, USB_DESC_STRING, 'K',0,'e',0,'y',0,'b',0,'o',0,'a',0,'r',0,'d',0};

// Constructor
SoftUSBKeyboard_RMT::SoftUSBKeyboard_RMT(uint8_t pinDp, uint8_t pinDm)
    : _pinDp(pinDp), _pinDm(pinDm), _configured(false), _address(0), _pendingAddress(0),
      _dataToggle(false), _reportChanged(false), _rxEnabled(false),
      _rxChannel(NULL), _txChannel(NULL), _txEncoder(NULL),
      _lastBitPos(0), _lastBitLevel(0),
      _txData(NULL), _txLen(0), _txOffset(0) {
    memset(_keyReport, 0, sizeof(_keyReport));
}

bool SoftUSBKeyboard_RMT::begin() {
    // Disable internal USB
    USB_SERIAL_JTAG.conf0.usb_pad_enable = 0;
    USB_SERIAL_JTAG.conf0.dp_pullup = 0;
    
    Serial.printf("SoftUSB_RMT: D+ on GPIO%d, D- on GPIO%d\n", _pinDp, _pinDm);
    
    // Configure D- with pullup for Low-Speed identification
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pin_bit_mask = (1ULL << _pinDm);
    gpio_config(&io_conf);
    
    // D+ will be controlled by RMT RX
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pin_bit_mask = (1ULL << _pinDp);
    gpio_config(&io_conf);
    
    // Initialize RMT RX only
    if (!initRMT()) {
        Serial.println("SoftUSB_RMT: Failed to initialize RMT");
        return false;
    }
    
    Serial.println("SoftUSB_RMT: RMT initialized");
    
    // Signal device attachment - SE0 then J state
    gpio_set_direction((gpio_num_t)_pinDp, GPIO_MODE_OUTPUT_OD);
    gpio_set_direction((gpio_num_t)_pinDm, GPIO_MODE_OUTPUT_OD);
    gpio_set_level((gpio_num_t)_pinDp, 0);
    gpio_set_level((gpio_num_t)_pinDm, 0);
    delayMicroseconds(10);
    gpio_set_level((gpio_num_t)_pinDm, 1); // J state for Low-Speed
    gpio_set_direction((gpio_num_t)_pinDp, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)_pinDm, GPIO_MODE_INPUT);
    
    Serial.println("SoftUSB_RMT: Device attached");
    Serial.printf("Initial state: D+=%d D-=%d\n", 
                  gpio_get_level((gpio_num_t)_pinDp),
                  gpio_get_level((gpio_num_t)_pinDm));
    
    return true;
}

bool SoftUSBKeyboard_RMT::initRMT() {
    esp_err_t ret;
    
    // Configure RMT RX channel on D+
    rmt_rx_channel_config_t rx_config = {};
    rx_config.gpio_num = (gpio_num_t)_pinDp;
    rx_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_config.resolution_hz = 80000000; // 80MHz = 12.5ns resolution
    rx_config.mem_block_symbols = 64;
    rx_config.flags.invert_in = false;
    rx_config.flags.with_dma = false;
    rx_config.flags.io_loop_back = false;
    
    ret = rmt_new_rx_channel(&rx_config, &_rxChannel);
    if (ret != ESP_OK) {
        Serial.printf("RMT RX channel creation failed: %d\n", ret);
        return false;
    }
    
    // Register RX done callback
    rmt_rx_event_callbacks_t rx_cbs = {};
    rx_cbs.on_recv_done = rmt_rx_done_callback;
    ret = rmt_rx_register_event_callbacks(_rxChannel, &rx_cbs, this);
    if (ret != ESP_OK) {
        Serial.printf("RMT RX callback registration failed: %d\n", ret);
        rmt_del_channel(_rxChannel);
        return false;
    }
    
    // Enable RX channel
    ret = rmt_enable(_rxChannel);
    if (ret != ESP_OK) {
        Serial.printf("RMT RX enable failed: %d\n", ret);
        return false;
    }
    _rxEnabled = true;
    
    Serial.println("RMT RX channel initialized");
    return true;
}

// Start RMT receive - always reset RMT state before starting new receive
bool SoftUSBKeyboard_RMT::startReceive() {
    // Always disable and re-enable to reset RMT state after previous receive
    if (_rxEnabled) {
        rmt_disable(_rxChannel);
        _rxEnabled = false;
    }
    
    esp_err_t ret = rmt_enable(_rxChannel);
    if (ret != ESP_OK) {
        return false;
    }
    _rxEnabled = true;
    
    rxDone = false;
    rxSymbolCount = 0;
    
    rmt_receive_config_t rx_config = {};
    rx_config.signal_range_min_ns = 100;    // Min signal width ~0.15 bit
    rx_config.signal_range_max_ns = 1500;   // Max ~2.2 bits - detect idle quickly
    
    ret = rmt_receive(_rxChannel, _rxBuffer, sizeof(_rxBuffer), &rx_config);
    return (ret == ESP_OK);
}

// Saved bit stream for multi-packet decode
static uint8_t savedBitStream[256];
static int savedBitCount = 0;
static int savedBitTicks = 53;

// Check if packet received
bool SoftUSBKeyboard_RMT::checkReceive(uint8_t* buffer, uint8_t* len) {
    if (!rxDone) return false;
    
    uint32_t now = esp_cpu_get_cycle_count();
    uint32_t latencyUs = (now - rxDoneTime) / 160;  // At 160MHz
    
    rxDone = false;
    
    if (rxSymbolCount < 4) return false;
    
    // Copy from ISR buffer
    memcpy(_rxBuffer, rxSymbolBuffer, rxSymbolCount * sizeof(rmt_symbol_word_t));
    
    static int debugLat = 0;
    if (debugLat++ < 5) {
        Serial.printf("RX latency: %lu us, symbols: %d\n", latencyUs, rxSymbolCount);
    }
    
    // Decode NRZI
    return decodeNRZI(_rxBuffer, rxSymbolCount, buffer, len);
}

// Decode second packet from saved bit stream (after SETUP token)
bool SoftUSBKeyboard_RMT::decodeNextPacket(uint8_t* data, uint8_t* len) {
    *len = 0;
    
    if (_lastBitPos <= 0 || _lastBitPos >= savedBitCount - 16) {
        return false;
    }
    
    int bitTicks = savedBitTicks;
    
    // Find next SYNC pattern starting from _lastBitPos
    int syncEnd = -1;
    for (int i = _lastBitPos + 7; i < savedBitCount; i++) {
        if (savedBitStream[i] == 1 && savedBitStream[i-1] == 1) {
            bool valid = true;
            for (int j = 0; j < 6; j++) {
                int idx = i - 2 - j;
                if (idx < _lastBitPos) { valid = false; break; }
                int expected = (j % 2 == 0) ? 0 : 1;
                if (savedBitStream[idx] != expected) { valid = false; break; }
            }
            if (valid) {
                syncEnd = i + 1;
                break;
            }
        }
    }
    
    if (syncEnd < 0 || syncEnd >= savedBitCount - 8) {
        return false;
    }
    
    // Decode NRZI for DATA packet (up to 11 bytes: PID + 8 data + 2 CRC)
    uint8_t byte = 0;
    int bitPos = 0;
    int onesCount = 0;
    uint8_t lastBit = 1;
    
    for (int i = syncEnd; i < savedBitCount && *len < 12; i++) {
        uint8_t dataBit;
        
        if (savedBitStream[i] == lastBit) {
            dataBit = 1;
            onesCount++;
            if (onesCount == 6) {
                onesCount = 0;
                if (i + 1 < savedBitCount) {
                    i++;
                    lastBit = savedBitStream[i];
                }
                continue;
            }
        } else {
            dataBit = 0;
            onesCount = 0;
        }
        
        lastBit = savedBitStream[i];
        byte |= (dataBit << bitPos);
        bitPos++;
        
        if (bitPos == 8) {
            data[(*len)++] = byte;
            byte = 0;
            bitPos = 0;
        }
    }
    
    return *len > 0;
}

// Decode NRZI from RMT symbols to USB data
// This handles the case where multiple packets are captured together
bool SoftUSBKeyboard_RMT::decodeNRZI(const rmt_symbol_word_t* symbols, size_t symbolCount, 
                                      uint8_t* data, uint8_t* len) {
    *len = 0;
    
    // Find bit time from first transitions (should be ~53 ticks)
    int bitTicks = RMT_USB_BIT_TICKS;
    if (symbolCount >= 2) {
        int firstDur = symbols[0].duration0;
        if (firstDur > 30 && firstDur < 80) {
            bitTicks = firstDur;
        }
    }
    int halfBit = bitTicks / 2;
    
    // Convert RMT symbols to bit stream with SE0 detection
    // SE0 appears as both D+ and D- low - in our D+ only capture,
    // it shows as extended low period (2 bit times)
    uint8_t bitStream[256];
    uint8_t bitLevel[256];  // Store the actual level for SE0 detection
    int bitCount = 0;
    
    for (size_t i = 0; i < symbolCount && bitCount < 256; i++) {
        // Part 0
        if (symbols[i].duration0 > 0) {
            int numBits = (symbols[i].duration0 + halfBit) / bitTicks;
            if (numBits < 1) numBits = 1;
            if (numBits > 8) numBits = 8;
            for (int b = 0; b < numBits && bitCount < 256; b++) {
                bitStream[bitCount] = symbols[i].level0;
                bitLevel[bitCount] = symbols[i].level0;
                bitCount++;
            }
        }
        
        // Part 1
        if (symbols[i].duration1 > 0) {
            int numBits = (symbols[i].duration1 + halfBit) / bitTicks;
            if (numBits < 1) numBits = 1;
            if (numBits > 8) numBits = 8;
            for (int b = 0; b < numBits && bitCount < 256; b++) {
                bitStream[bitCount] = symbols[i].level1;
                bitLevel[bitCount] = symbols[i].level1;
                bitCount++;
            }
        }
    }
    
    if (bitCount < 16) return false;
    
    // Save bit stream for potential multi-packet decode
    memcpy(savedBitStream, bitStream, bitCount);
    savedBitCount = bitCount;
    savedBitTicks = bitTicks;
    _lastBitPos = 0;
    
    // Debug
    static int debugCnt = 0;
    if (debugCnt++ < 10) {
        Serial.printf("RMT bits=%d, bitTicks=%d: ", bitCount, bitTicks);
        for (int i = 0; i < 32 && i < bitCount; i++) {
            Serial.printf("%d", bitStream[i]);
        }
        Serial.println();
    }
    
    // Find first SYNC pattern (KJKJKJKK on D+: 10101011)
    int syncEnd = -1;
    for (int i = 7; i < bitCount; i++) {
        // Check for SYNC ending with KK (11)
        if (bitStream[i] == 1 && bitStream[i-1] == 1) {
            // Verify alternating before
            bool valid = true;
            for (int j = 0; j < 6; j++) {
                int idx = i - 2 - j;
                if (idx < 0) { valid = false; break; }
                int expected = (j % 2 == 0) ? 0 : 1;
                if (bitStream[idx] != expected) { valid = false; break; }
            }
            if (valid) {
                syncEnd = i + 1;
                break;
            }
        }
    }
    
    if (syncEnd < 0 || syncEnd >= bitCount - 8) return false;
    
    // Decode NRZI starting after SYNC
    uint8_t byte = 0;
    int bitPos = 0;
    int onesCount = 0;
    uint8_t lastBit = 1; // SYNC ends with K state (1 on D+)
    
    for (int i = syncEnd; i < bitCount && *len < 16; i++) {
        // Check for SE0 (EOP) - extended low period
        // In our stream, SE0 would appear as 2+ consecutive 0s after valid data
        // But we primarily detect it by packet length for token packets
        
        uint8_t dataBit;
        
        if (bitStream[i] == lastBit) {
            dataBit = 1; // No transition = 1
            onesCount++;
            
            // Bit stuffing: after 6 ones, next is a stuffed 0
            if (onesCount == 6) {
                onesCount = 0;
                if (i + 1 < bitCount) {
                    i++; // Skip stuffed bit
                    lastBit = bitStream[i];
                }
                continue;
            }
        } else {
            dataBit = 0; // Transition = 0
            onesCount = 0;
        }
        
        lastBit = bitStream[i];
        
        // Build byte LSB first
        byte |= (dataBit << bitPos);
        bitPos++;
        
        if (bitPos == 8) {
            data[(*len)++] = byte;
            byte = 0;
            bitPos = 0;
            
            // For token packets (SETUP/IN/OUT), stop after 3 bytes
            // PID tells us the packet type
            if (*len == 1) {
                uint8_t pid = data[0];
                if (pid == USB_PID_SETUP || pid == USB_PID_IN || pid == USB_PID_OUT) {
                    // Token packet is 3 bytes: PID + ADDR(7) + ENDP(4) + CRC5(5)
                    // Continue to get 2 more bytes
                }
            } else if (*len == 3) {
                // Check if this was a token packet
                uint8_t pid = data[0];
                if (pid == USB_PID_SETUP || pid == USB_PID_IN || pid == USB_PID_OUT) {
                    // Token packet complete - save position for potential DATA packet
                    _lastBitPos = i + 1;
                    _lastBitLevel = lastBit;
                    return true;
                }
            }
        }
    }
    
    return *len > 0;
}

// Delay for one USB bit (666.67ns at Low-Speed)
inline void IRAM_ATTR delayBit() {
    uint32_t start = esp_cpu_get_cycle_count();
    while ((esp_cpu_get_cycle_count() - start) < 107) {} // ~667ns at 160MHz
}

inline void IRAM_ATTR delayHalfBit() {
    uint32_t start = esp_cpu_get_cycle_count();
    while ((esp_cpu_get_cycle_count() - start) < 53) {}
}

// Fast polling-based USB receive with immediate ACK response
// Returns: 0 = no packet, 1 = got token, 2 = got token+data (SETUP)
static uint8_t IRAM_ATTR fastReceiveAndRespond(uint8_t pinDp, uint8_t pinDm, 
                                                 uint8_t* tokenBuf, uint8_t* tokenLen,
                                                 uint8_t* dataBuf, uint8_t* dataLen) {
    uint32_t dpMask = (1 << pinDp);
    uint32_t dmMask = (1 << pinDm);
    volatile uint32_t* inReg = &GPIO.in.val;
    
    *tokenLen = 0;
    *dataLen = 0;
    
    // Wait for activity (K state = D+ low, D- high for Low-Speed)
    // Idle is J state = D+ high, D- low... wait, for Low-Speed idle is K state?
    // Actually for Low-Speed: Idle = J = D- high, D+ low
    // So activity starts with K = D- low, D+ high (transition from J)
    
    // Wait for D+ to go high (start of K after J idle)
    uint32_t timeout = 1000;
    while (!((*inReg) & dpMask)) {
        if (--timeout == 0) return 0;
    }
    
    // Record edges
    uint16_t edges[96];
    uint8_t levels[96];
    uint8_t edgeCount = 0;
    uint32_t lastVal = (*inReg) & dpMask;
    uint32_t startCycle = esp_cpu_get_cycle_count();
    
    edges[edgeCount] = 0;
    levels[edgeCount++] = lastVal ? 1 : 0;
    
    timeout = 1500;  // About 10us at 160MHz
    while (edgeCount < 95 && timeout > 0) {
        uint32_t val = (*inReg) & dpMask;
        if (val != lastVal) {
            edges[edgeCount] = (esp_cpu_get_cycle_count() - startCycle) >> 3;
            levels[edgeCount++] = val ? 1 : 0;
            lastVal = val;
            timeout = 1500;
        } else {
            timeout--;
        }
    }
    
    if (edgeCount < 8) return 0;  // Too short
    
    // Calculate bit time
    int bitTime = 13;  // Default ~107/8
    int count = 0;
    for (int i = 1; i < 6 && i < edgeCount; i++) {
        int delta = edges[i] - edges[i-1];
        if (delta > 5 && delta < 25) {
            bitTime += delta;
            count++;
        }
    }
    if (count > 0) bitTime /= count;
    
    // Decode NRZI - find sync, then decode bytes
    // Looking for 8 alternating bits ending with same (10101011)
    int syncEnd = 0;
    for (int i = 7; i < edgeCount; i++) {
        if (levels[i] == levels[i-1]) {  // Two same levels = end of sync
            // Verify alternating before
            bool ok = true;
            for (int j = i-7; j < i-1; j++) {
                if (levels[j] == levels[j+1]) { ok = false; break; }
            }
            if (ok) {
                syncEnd = i + 1;
                break;
            }
        }
    }
    
    if (syncEnd == 0) return 0;
    
    // Decode first packet (token)
    uint8_t byte = 0;
    int bitPos = 0;
    int onesCount = 0;
    uint8_t lastLevel = levels[syncEnd - 1];
    
    for (int i = syncEnd; i < edgeCount && *tokenLen < 4; i++) {
        int delta = (i > 0) ? (edges[i] - edges[i-1]) : bitTime;
        int bits = (delta + bitTime/2) / bitTime;
        if (bits < 1) bits = 1;
        if (bits > 6) bits = 6;
        
        // Each edge = transition = 0 bit
        // Between edges = bits-1 same bits = 1s
        
        // First, add the 1 bits (no transitions)
        for (int b = 0; b < bits - 1 && *tokenLen < 4; b++) {
            onesCount++;
            if (onesCount == 6) {
                onesCount = 0;
                continue;  // Skip stuffed bit
            }
            byte |= (1 << bitPos);
            bitPos++;
            if (bitPos == 8) {
                tokenBuf[(*tokenLen)++] = byte;
                byte = 0;
                bitPos = 0;
            }
        }
        
        // Then the 0 bit (transition)
        if (*tokenLen < 4) {
            onesCount = 0;
            bitPos++;
            if (bitPos == 8) {
                tokenBuf[(*tokenLen)++] = byte;
                byte = 0;
                bitPos = 0;
            }
        }
        
        lastLevel = levels[i];
    }
    
    return (*tokenLen >= 1) ? 1 : 0;
}

// Send USB bit using direct GPIO (bit-bang for TX)
void IRAM_ATTR SoftUSBKeyboard_RMT::sendBit(bool dpLevel, bool dmLevel) {
    if (dpLevel) GPIO.out_w1ts.val = (1 << _pinDp);
    else GPIO.out_w1tc.val = (1 << _pinDp);
    
    if (dmLevel) GPIO.out_w1ts.val = (1 << _pinDm);
    else GPIO.out_w1tc.val = (1 << _pinDm);
    
    delayBit();
}

// Send USB packet with bit-banging
void SoftUSBKeyboard_RMT::sendPacket(const uint8_t* data, uint8_t len) {
    // Disable RMT RX during TX
    if (_rxEnabled) {
        rmt_disable(_rxChannel);
        _rxEnabled = false;
    }
    
    // Set GPIO to output mode
    gpio_set_direction((gpio_num_t)_pinDp, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)_pinDm, GPIO_MODE_OUTPUT);
    
    portDISABLE_INTERRUPTS();
    
    // For Low-Speed: K = D- high, D+ low; J = D- low, D+ high
    // SYNC pattern: KJKJKJKK (8 bits)
    sendBit(0, 1); // K
    sendBit(1, 0); // J
    sendBit(0, 1); // K
    sendBit(1, 0); // J
    sendBit(0, 1); // K
    sendBit(1, 0); // J
    sendBit(0, 1); // K
    sendBit(0, 1); // K
    
    // Send data with NRZI encoding
    bool lastDp = false; // After SYNC, last state is K (D+ = 0)
    uint8_t onesCount = 0;
    
    for (int i = 0; i < len; i++) {
        for (int b = 0; b < 8; b++) {
            uint8_t bit = (data[i] >> b) & 1;
            
            if (bit == 1) {
                // No transition for 1
                onesCount++;
                sendBit(lastDp, !lastDp);
                
                // Bit stuffing after 6 ones
                if (onesCount == 6) {
                    // Insert 0 (transition)
                    lastDp = !lastDp;
                    sendBit(lastDp, !lastDp);
                    onesCount = 0;
                }
            } else {
                // Transition for 0
                lastDp = !lastDp;
                sendBit(lastDp, !lastDp);
                onesCount = 0;
            }
        }
    }
    
    // EOP: SE0 for 2 bit times, then J (idle for Low-Speed = D- high)
    sendBit(0, 0); // SE0
    sendBit(0, 0); // SE0
    sendBit(0, 1); // J for Low-Speed (D- high, D+ low)
    
    portENABLE_INTERRUPTS();
    
    // Return to input mode
    gpio_set_direction((gpio_num_t)_pinDp, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)_pinDm, GPIO_MODE_INPUT);
    
    // Don't re-enable RMT here - let startReceive() handle it
    // _rxEnabled stays false, startReceive() will enable it
}

void SoftUSBKeyboard_RMT::sendHandshake(uint8_t pid) {
    uint8_t packet[1] = {pid};
    sendPacket(packet, 1);
}

void SoftUSBKeyboard_RMT::sendDataPacket(uint8_t pid, const uint8_t* data, uint8_t len) {
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

uint8_t SoftUSBKeyboard_RMT::crc5(uint16_t data) {
    uint8_t crc = 0x1F;
    for (int i = 0; i < 11; i++) {
        uint8_t bit = (data >> i) & 1;
        if ((crc & 1) ^ bit) crc = (crc >> 1) ^ 0x14;
        else crc >>= 1;
    }
    return crc ^ 0x1F;
}

uint16_t SoftUSBKeyboard_RMT::crc16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    if (data) {
        for (int i = 0; i < len; i++) {
            for (int b = 0; b < 8; b++) {
                if ((crc & 1) ^ ((data[i] >> b) & 1)) crc = (crc >> 1) ^ 0xA001;
                else crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFF;
}

void SoftUSBKeyboard_RMT::task() {
    static uint32_t lastDebug = 0;
    static uint32_t packetCount = 0;
    static bool receiving = false;
    
    // Debug output
    if (millis() - lastDebug > 2000) {
        lastDebug = millis();
        Serial.printf("USB_RMT: D+=%d D-=%d, addr=%d, cfg=%d, pkts=%lu\n",
                      gpio_get_level((gpio_num_t)_pinDp),
                      gpio_get_level((gpio_num_t)_pinDm),
                      _address, _configured ? 1 : 0,
                      packetCount);
    }
    
    // Start receive if not already
    if (!receiving) {
        if (startReceive()) {
            receiving = true;
        }
    }
    
    // Check for received packet
    uint8_t buffer[64];
    uint8_t len;
    
    if (receiving && checkReceive(buffer, &len)) {
        receiving = false;
        packetCount++;
        
        if (len < 1) {
            return;
        }
        
        uint8_t pid = buffer[0];
        
        switch (pid) {
            case USB_PID_SETUP: {
                // SETUP token received - DATA0 should be in same capture
                uint8_t dataBuffer[16];
                uint8_t dataLen;
                
                uint32_t ackStart = esp_cpu_get_cycle_count();
                
                if (decodeNextPacket(dataBuffer, &dataLen)) {
                    if (dataLen >= 9 && dataBuffer[0] == USB_PID_DATA0) {
                        // Send ACK immediately, then process
                        sendHandshake(USB_PID_ACK);
                        
                        uint32_t ackEnd = esp_cpu_get_cycle_count();
                        uint32_t ackUs = (ackEnd - ackStart) / 160;
                        uint32_t totalUs = (ackEnd - rxDoneTime) / 160;
                        Serial.printf("ACK sent: decode=%luus total=%luus\n", ackUs, totalUs);
                        
                        handleSetupPacket(&dataBuffer[1]);
                        
                        Serial.printf("SETUP: type=%02X req=%02X\n", 
                                      dataBuffer[1], dataBuffer[2]);
                    }
                } else {
                    // DATA0 not in same capture, wait for it separately
                    delayMicroseconds(5);
                    startReceive();
                    for (int i = 0; i < 100; i++) {
                        delayMicroseconds(10);
                        if (checkReceive(buffer, &len)) {
                            if (len >= 9 && buffer[0] == USB_PID_DATA0) {
                                sendHandshake(USB_PID_ACK);
                                handleSetupPacket(&buffer[1]);
                            }
                            break;
                        }
                    }
                }
                break;
            }
                
            case USB_PID_IN:
                // Check if we have pending data to send (from SETUP request)
                if (_txData != NULL && _txLen > 0) {
                    // Send next chunk of pending data
                    uint8_t chunkLen = _txLen - _txOffset;
                    if (chunkLen > 8) chunkLen = 8;  // Low-Speed max packet size
                    
                    sendDataPacket(_dataToggle ? USB_PID_DATA1 : USB_PID_DATA0,
                                   _txData + _txOffset, chunkLen);
                    _dataToggle = !_dataToggle;
                    _txOffset += chunkLen;
                    
                    // If all data sent, clear pending
                    if (_txOffset >= _txLen) {
                        _txData = NULL;
                        _txLen = 0;
                        _txOffset = 0;
                    }
                } else if (_configured && _reportChanged) {
                    // Send HID report
                    sendDataPacket(_dataToggle ? USB_PID_DATA1 : USB_PID_DATA0,
                                   _keyReport, 8);
                    _dataToggle = !_dataToggle;
                    _reportChanged = false;
                } else {
                    sendHandshake(USB_PID_NAK);
                }
                break;
                
            case USB_PID_OUT: {
                // Wait for data packet
                uint8_t dataBuffer[16];
                uint8_t dataLen;
                bool gotData = false;
                
                // First try from same capture
                if (decodeNextPacket(dataBuffer, &dataLen) && dataLen >= 1) {
                    gotData = true;
                } else {
                    // Wait for separate capture
                    delayMicroseconds(5);
                    startReceive();
                    for (int i = 0; i < 100; i++) {
                        delayMicroseconds(10);
                        if (checkReceive(dataBuffer, &dataLen)) {
                            gotData = true;
                            break;
                        }
                    }
                }
                
                if (gotData) {
                    sendHandshake(USB_PID_ACK);
                    // Check if we have a pending address change (after SET_ADDRESS status stage)
                    if (_pendingAddress != 0) {
                        _address = _pendingAddress;
                        _pendingAddress = 0;
                        Serial.printf("Address now: %d\n", _address);
                    }
                }
                break;
            }
                break;
                
            default:
                break;
        }
    }
}

void SoftUSBKeyboard_RMT::handleSetupPacket(const uint8_t* setup) {
    uint8_t bmRequestType = setup[0];
    uint8_t bRequest = setup[1];
    uint16_t wValue = setup[2] | (setup[3] << 8);
    uint16_t wIndex = setup[4] | (setup[5] << 8);
    uint16_t wLength = setup[6] | (setup[7] << 8);
    
    Serial.printf("SETUP: type=%02X req=%02X val=%04X idx=%04X len=%04X\n",
                  bmRequestType, bRequest, wValue, wIndex, wLength);
    
    _dataToggle = true; // After SETUP, first DATA is DATA1
    _txData = NULL;
    _txLen = 0;
    _txOffset = 0;
    
    // Static buffer for small responses
    static uint8_t statusBuffer[2] = {0, 0};
    
    if (bmRequestType == 0x80) {
        // Device to Host
        if (bRequest == 0x06) {
            // GET_DESCRIPTOR - prepare data for IN token
            handleGetDescriptor((wValue >> 8) & 0xFF, wValue & 0xFF, wLength);
        } else if (bRequest == 0x00) {
            // GET_STATUS - prepare status for IN token
            statusBuffer[0] = 0;
            statusBuffer[1] = 0;
            _txData = statusBuffer;
            _txLen = 2;
            _txOffset = 0;
        }
    } else if (bmRequestType == 0x00) {
        // Host to Device (no data stage, or OUT data stage)
        if (bRequest == 0x05) {
            // SET_ADDRESS - will send zero-length DATA on IN token
            // Address changes after status stage completes
            _pendingAddress = wValue & 0x7F;
            _txData = statusBuffer;  // Zero-length packet
            _txLen = 0;
            _txOffset = 0;
            Serial.printf("SET_ADDRESS: %d (pending)\n", _pendingAddress);
        } else if (bRequest == 0x09) {
            // SET_CONFIGURATION - respond on IN token with zero-length
            _configured = true;
            _txData = statusBuffer;
            _txLen = 0;
            _txOffset = 0;
            Serial.println("Device configured!");
        }
    } else if (bmRequestType == 0x21) {
        // Class request (HID) - Host to Device
        if (bRequest == 0x0A) {
            // SET_IDLE - respond on IN token with zero-length
            _txData = statusBuffer;
            _txLen = 0;
            _txOffset = 0;
        }
    } else if (bmRequestType == 0x81) {
        // Interface to Host
        if (bRequest == 0x06) {
            // GET_DESCRIPTOR (HID specific)
            handleGetDescriptor((wValue >> 8) & 0xFF, wValue & 0xFF, wLength);
        }
    }
}

void SoftUSBKeyboard_RMT::handleGetDescriptor(uint8_t type, uint8_t index, uint16_t length) {
    const uint8_t* desc = nullptr;
    uint8_t descLen = 0;
    
    Serial.printf("GetDesc: type=%02X idx=%d len=%d\n", type, index, length);
    
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
            if (index == 0) { desc = stringDescLang; descLen = sizeof(stringDescLang); }
            else if (index == 1) { desc = stringDescMfg; descLen = sizeof(stringDescMfg); }
            else if (index == 2) { desc = stringDescProd; descLen = sizeof(stringDescProd); }
            break;
        case USB_DESC_HID_REPORT:
            desc = hidReportDescriptor;
            descLen = sizeof(hidReportDescriptor);
            break;
    }
    
    if (desc && descLen > 0) {
        // Prepare data for IN token - don't send immediately
        _txData = desc;
        _txLen = (descLen < length) ? descLen : length;
        _txOffset = 0;
        Serial.printf("Prepared %d bytes for IN\n", _txLen);
    }
}

void SoftUSBKeyboard_RMT::pressKey(uint8_t keycode) {
    for (int i = 2; i < 8; i++) {
        if (_keyReport[i] == 0) {
            _keyReport[i] = keycode;
            _reportChanged = true;
            break;
        }
    }
}

void SoftUSBKeyboard_RMT::releaseKey(uint8_t keycode) {
    for (int i = 2; i < 8; i++) {
        if (_keyReport[i] == keycode) {
            _keyReport[i] = 0;
            _reportChanged = true;
            break;
        }
    }
}

void SoftUSBKeyboard_RMT::releaseAll() {
    memset(_keyReport, 0, sizeof(_keyReport));
    _reportChanged = true;
}
