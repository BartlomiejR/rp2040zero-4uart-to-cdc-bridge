#include <cstring>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "tusb.h"

#include "pio_uart.pio.h"

// --- Pin assignments ---
// Port 0: Hardware UART0 - YDLidar TMini Plus
#define PORT0_TX_PIN  0
#define PORT0_RX_PIN  1

// Port 1: Hardware UART1 - DDSM115
#define PORT1_TX_PIN  4
#define PORT1_RX_PIN  5

// Port 2: PIO UART - IMU (high speed)
#define PORT2_TX_PIN  8
#define PORT2_RX_PIN  9

// Port 3: PIO UART - Aux
#define PORT3_TX_PIN  12
#define PORT3_RX_PIN  13

// --- Default baud rates ---
#define DEFAULT_BAUD      115200
#define BAUD_LIDAR        230400  // Port 0: YDLidar TMini Plus
#define BAUD_IMU          230400  // Port 2: IMU

// --- PIO state ---
static PIO pio_uart_pio = pio0;

// PIO state machine assignments (all on PIO0)
static uint pio_tx_sm[2];  // SM for PIO UART 2 and 3 TX
static uint pio_rx_sm[2];  // SM for PIO UART 2 and 3 RX
static uint pio_tx_offset;
static uint pio_rx_offset;

// Current baud rates
static uint32_t port_baud[4] = {BAUD_LIDAR, DEFAULT_BAUD, BAUD_IMU, DEFAULT_BAUD};

// --- Intermediate buffers for USB<->UART bridging ---
static uint8_t bridge_buf[64];

// --- Init functions ---

static void hw_uart_init_port(uart_inst_t *uart, uint tx_pin, uint rx_pin, uint baud) {
    uart_init(uart, baud);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    // Disable UART FIFO so we get data byte-by-byte (lower latency)
    // Actually keep FIFO enabled for throughput
    uart_set_fifo_enabled(uart, true);
}

static void pio_uart_init_port(uint port_idx, uint tx_pin, uint rx_pin, uint baud) {
    // port_idx: 0 or 1 (maps to PIO UART 2 or 3)
    pio_tx_sm[port_idx] = pio_claim_unused_sm(pio_uart_pio, true);
    pio_rx_sm[port_idx] = pio_claim_unused_sm(pio_uart_pio, true);

    pio_uart_tx_program_init(pio_uart_pio, pio_tx_sm[port_idx], pio_tx_offset, tx_pin, baud);
    pio_uart_rx_program_init(pio_uart_pio, pio_rx_sm[port_idx], pio_rx_offset, rx_pin, baud);
}

// --- Baud rate update ---

static void set_port_baud(uint8_t port, uint32_t baud) {
    if (baud == 0) return;
    port_baud[port] = baud;

    switch (port) {
        case 0:
            uart_set_baudrate(uart0, baud);
            break;
        case 1:
            uart_set_baudrate(uart1, baud);
            break;
        case 2:
            pio_uart_tx_set_baud(pio_uart_pio, pio_tx_sm[0], baud);
            pio_uart_rx_set_baud(pio_uart_pio, pio_rx_sm[0], baud);
            break;
        case 3:
            pio_uart_tx_set_baud(pio_uart_pio, pio_tx_sm[1], baud);
            pio_uart_rx_set_baud(pio_uart_pio, pio_rx_sm[1], baud);
            break;
    }
}

// --- TinyUSB CDC callbacks ---

extern "C" {

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *p_line_coding) {
    if (itf < 4) {
        set_port_baud(itf, p_line_coding->bit_rate);

        // For hardware UARTs, also apply data bits, parity, stop bits
        if (itf < 2) {
            uart_inst_t *uart = (itf == 0) ? uart0 : uart1;

            uint data_bits;
            switch (p_line_coding->data_bits) {
                case 5: data_bits = 5; break;
                case 6: data_bits = 6; break;
                case 7: data_bits = 7; break;
                default: data_bits = 8; break;
            }

            uint stop_bits;
            switch (p_line_coding->stop_bits) {
                case 2: stop_bits = 2; break;
                default: stop_bits = 1; break;
            }

            uart_parity_t parity;
            switch (p_line_coding->parity) {
                case 1: parity = UART_PARITY_ODD; break;
                case 2: parity = UART_PARITY_EVEN; break;
                default: parity = UART_PARITY_NONE; break;
            }

            uart_set_format(uart, data_bits, stop_bits, parity);
        }
    }
}

} // extern "C"

// --- Bridge data transfer ---

// USB CDC -> Hardware UART
static void bridge_usb_to_hw_uart(uint8_t cdc_itf, uart_inst_t *uart) {
    if (!tud_cdc_n_available(cdc_itf)) return;

    uint32_t count = tud_cdc_n_read(cdc_itf, bridge_buf, sizeof(bridge_buf));
    for (uint32_t i = 0; i < count; i++) {
        uart_putc_raw(uart, bridge_buf[i]);
    }
}

// Hardware UART -> USB CDC
static void bridge_hw_uart_to_usb(uint8_t cdc_itf, uart_inst_t *uart) {
    uint32_t count = 0;
    while (uart_is_readable(uart) && count < sizeof(bridge_buf)) {
        bridge_buf[count++] = uart_getc(uart);
    }
    if (count > 0 && tud_cdc_n_write_available(cdc_itf)) {
        tud_cdc_n_write(cdc_itf, bridge_buf, count);
        tud_cdc_n_write_flush(cdc_itf);
    }
}

// USB CDC -> PIO UART TX
static void bridge_usb_to_pio_uart(uint8_t cdc_itf, uint pio_idx) {
    if (!tud_cdc_n_available(cdc_itf)) return;

    uint32_t count = tud_cdc_n_read(cdc_itf, bridge_buf, sizeof(bridge_buf));
    for (uint32_t i = 0; i < count; i++) {
        // Blocking put - PIO TX FIFO is 8-deep with joining
        pio_sm_put_blocking(pio_uart_pio, pio_tx_sm[pio_idx], (uint32_t)bridge_buf[i]);
    }
}

// PIO UART RX -> USB CDC
static void bridge_pio_uart_to_usb(uint8_t cdc_itf, uint pio_idx) {
    uint32_t count = 0;
    while (!pio_sm_is_rx_fifo_empty(pio_uart_pio, pio_rx_sm[pio_idx]) && count < sizeof(bridge_buf)) {
        bridge_buf[count++] = (uint8_t)(pio_sm_get(pio_uart_pio, pio_rx_sm[pio_idx]) >> 24);
    }
    if (count > 0 && tud_cdc_n_write_available(cdc_itf)) {
        tud_cdc_n_write(cdc_itf, bridge_buf, count);
        tud_cdc_n_write_flush(cdc_itf);
    }
}

// --- Main ---

int main() {
    // Initialize hardware UARTs
    hw_uart_init_port(uart0, PORT0_TX_PIN, PORT0_RX_PIN, BAUD_LIDAR);
    hw_uart_init_port(uart1, PORT1_TX_PIN, PORT1_RX_PIN, DEFAULT_BAUD);

    // Load PIO programs
    pio_tx_offset = pio_add_program(pio_uart_pio, &pio_uart_tx_program);
    pio_rx_offset = pio_add_program(pio_uart_pio, &pio_uart_rx_program);

    // Initialize PIO UARTs
    pio_uart_init_port(0, PORT2_TX_PIN, PORT2_RX_PIN, BAUD_IMU);
    pio_uart_init_port(1, PORT3_TX_PIN, PORT3_RX_PIN, DEFAULT_BAUD);

    // Initialize TinyUSB
    tusb_init();

    while (true) {
        tud_task();

        // Bridge all 4 ports
        // Port 0: HW UART0 <-> CDC 0
        bridge_usb_to_hw_uart(0, uart0);
        bridge_hw_uart_to_usb(0, uart0);

        // Port 1: HW UART1 <-> CDC 1
        bridge_usb_to_hw_uart(1, uart1);
        bridge_hw_uart_to_usb(1, uart1);

        // Port 2: PIO UART 0 <-> CDC 2
        bridge_usb_to_pio_uart(2, 0);
        bridge_pio_uart_to_usb(2, 0);

        // Port 3: PIO UART 1 <-> CDC 3
        bridge_usb_to_pio_uart(3, 1);
        bridge_pio_uart_to_usb(3, 1);
    }
}
