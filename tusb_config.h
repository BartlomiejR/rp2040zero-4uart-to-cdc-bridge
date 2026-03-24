#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#define CFG_TUSB_MCU            OPT_MCU_RP2040
#define CFG_TUSB_OS             OPT_OS_PICO
#define CFG_TUSB_RHPORT0_MODE   OPT_MODE_DEVICE
#define CFG_TUD_ENABLED         1

// 5 CDC interfaces - four UART bridges plus one diagnostics console
#define CFG_TUD_CDC         5
#define CFG_TUD_CDC_RX_BUFSIZE  512
#define CFG_TUD_CDC_TX_BUFSIZE  1024

// Disable unused classes
#define CFG_TUD_MSC         0
#define CFG_TUD_HID         0
#define CFG_TUD_MIDI        0
#define CFG_TUD_VENDOR      0

#endif
