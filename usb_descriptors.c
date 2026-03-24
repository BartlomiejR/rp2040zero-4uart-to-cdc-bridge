#include "tusb.h"
#include "pico/unique_id.h"

// Device descriptor - IAD composite device
static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,          // IAD composite
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x2E8A,   // Raspberry Pi
    .idProduct          = 0x4004,   // Custom PID for 4-port bridge
    .bcdDevice          = 0x0100,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1,
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

// Configuration descriptor
// Each CDC function uses 2 interfaces and 3 endpoints
// Endpoint allocation:
//   CDC 0: notif=0x81, out=0x02, in=0x82
//   CDC 1: notif=0x84, out=0x05, in=0x85
//   CDC 2: notif=0x87, out=0x08, in=0x88
//   CDC 3: notif=0x8A, out=0x0B, in=0x8B

enum {
    ITF_NUM_CDC0 = 0,
    ITF_NUM_CDC0_DATA,
    ITF_NUM_CDC1,
    ITF_NUM_CDC1_DATA,
    ITF_NUM_CDC2,
    ITF_NUM_CDC2_DATA,
    ITF_NUM_CDC3,
    ITF_NUM_CDC3_DATA,
    ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + 4 * TUD_CDC_DESC_LEN)

#define EPNUM_CDC0_NOTIF  0x81
#define EPNUM_CDC0_OUT    0x02
#define EPNUM_CDC0_IN     0x82

#define EPNUM_CDC1_NOTIF  0x84
#define EPNUM_CDC1_OUT    0x05
#define EPNUM_CDC1_IN     0x85

#define EPNUM_CDC2_NOTIF  0x87
#define EPNUM_CDC2_OUT    0x08
#define EPNUM_CDC2_IN     0x88

#define EPNUM_CDC3_NOTIF  0x8A
#define EPNUM_CDC3_OUT    0x0B
#define EPNUM_CDC3_IN     0x8B

static const uint8_t desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          0x00, 500),

    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC0, 4, EPNUM_CDC0_NOTIF, 8,
                       EPNUM_CDC0_OUT, EPNUM_CDC0_IN, 64),

    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC1, 5, EPNUM_CDC1_NOTIF, 8,
                       EPNUM_CDC1_OUT, EPNUM_CDC1_IN, 64),

    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC2, 6, EPNUM_CDC2_NOTIF, 8,
                       EPNUM_CDC2_OUT, EPNUM_CDC2_IN, 64),

    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC3, 7, EPNUM_CDC3_NOTIF, 8,
                       EPNUM_CDC3_OUT, EPNUM_CDC3_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

// String descriptors
static const char *const string_desc_arr[] = {
    [0] = (const char[]){0x09, 0x04},  // English (US)
    [1] = "RP2040 Bridge",             // Manufacturer
    [2] = "4-Port UART Bridge",        // Product
    [3] = NULL,                        // Serial (generated at runtime)
    [4] = "UART0 - LiDAR",              // CDC 0
    [5] = "UART1 - DDSM115",            // CDC 1
    [6] = "UART2 - IMU",               // CDC 2
    [7] = "UART3 - Aux",               // CDC 3
};

static uint16_t _desc_str[33];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;

    if (index == 0) {
        _desc_str[1] = 0x0409;  // English (US)
        chr_count = 1;
    } else if (index == 3) {
        // Serial number from unique board ID
        pico_unique_board_id_t id;
        pico_get_unique_board_id(&id);
        chr_count = 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES;
        if (chr_count > 32) chr_count = 32;
        for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
            uint8_t nibble_h = (id.id[i] >> 4) & 0x0F;
            uint8_t nibble_l = id.id[i] & 0x0F;
            _desc_str[1 + 2 * i]     = nibble_h < 10 ? '0' + nibble_h : 'A' + nibble_h - 10;
            _desc_str[1 + 2 * i + 1] = nibble_l < 10 ? '0' + nibble_l : 'A' + nibble_l - 10;
        }
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) return NULL;
        const char *str = string_desc_arr[index];
        if (!str) return NULL;
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 32) chr_count = 32;
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
