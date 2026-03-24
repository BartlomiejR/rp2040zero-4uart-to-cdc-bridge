#include <cstdint>
#include <cstdio>
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

#define BRIDGE_PORT_COUNT 4
#define TOTAL_CDC_COUNT   5
#define DIAG_CDC_ITF      4
#define DIAG_REPORT_INTERVAL_MS 1000

// --- PIO state ---
static PIO pio_uart_pio = pio0;

// PIO state machine assignments (all on PIO0)
static uint pio_tx_sm[2];  // SM for PIO UART 2 and 3 TX
static uint pio_rx_sm[2];  // SM for PIO UART 2 and 3 RX
static uint pio_tx_offset;
static uint pio_rx_offset;

// Current baud rates
static uint32_t port_baud[BRIDGE_PORT_COUNT] = {BAUD_LIDAR, DEFAULT_BAUD, BAUD_IMU, DEFAULT_BAUD};

// --- Intermediate buffers for USB<->UART bridging ---
static uint8_t bridge_buf[64];
static char diag_report_buf[1024];

static const char *const port_names[BRIDGE_PORT_COUNT] = {
    "LiDAR",
    "DDSM115",
    "IMU",
    "Aux",
};

struct uart_bridge_stats_t {
    uint64_t usb_to_uart_bytes;
    uint64_t uart_to_usb_bytes;
    uint64_t uart_to_usb_drop_bytes;
};

struct cdc_stats_t {
    cdc_line_coding_t line_coding;
    uint64_t rx_bytes;
    uint64_t tx_bytes;
    uint64_t tx_drop_bytes;
    bool connected;
    bool dtr;
    bool rts;
};

struct report_snapshot_t {
    uint64_t usb_to_uart_bytes;
    uint64_t uart_to_usb_bytes;
    uint64_t uart_to_usb_drop_bytes;
    uint64_t cdc_rx_bytes;
    uint64_t cdc_tx_bytes;
    uint64_t cdc_tx_drop_bytes;
};

static uart_bridge_stats_t port_stats[BRIDGE_PORT_COUNT];
static cdc_stats_t cdc_stats[TOTAL_CDC_COUNT] = {
    {{BAUD_LIDAR, 0, 0, 8}, 0, 0, 0, false, false, false},
    {{DEFAULT_BAUD, 0, 0, 8}, 0, 0, 0, false, false, false},
    {{BAUD_IMU, 0, 0, 8}, 0, 0, 0, false, false, false},
    {{DEFAULT_BAUD, 0, 0, 8}, 0, 0, 0, false, false, false},
    {{DEFAULT_BAUD, 0, 0, 8}, 0, 0, 0, false, false, false},
};
static report_snapshot_t last_report[BRIDGE_PORT_COUNT];
static uint32_t next_diag_report_ms;
static uint32_t diag_report_seq;

// --- Init functions ---

static const char *parity_to_str(uint8_t parity) {
    switch (parity) {
        case 1: return "O";
        case 2: return "E";
        case 3: return "M";
        case 4: return "S";
        default: return "N";
    }
}

static const char *stop_bits_to_str(uint8_t stop_bits) {
    switch (stop_bits) {
        case 1: return "1.5";
        case 2: return "2";
        default: return "1";
    }
}

static bool uart_rx_pending(uint8_t port) {
    switch (port) {
        case 0: return uart_is_readable(uart0);
        case 1: return uart_is_readable(uart1);
        case 2: return !pio_sm_is_rx_fifo_empty(pio_uart_pio, pio_rx_sm[0]);
        case 3: return !pio_sm_is_rx_fifo_empty(pio_uart_pio, pio_rx_sm[1]);
        default: return false;
    }
}

static void refresh_cdc_connection_state() {
    for (uint8_t itf = 0; itf < TOTAL_CDC_COUNT; ++itf) {
        cdc_stats[itf].connected = tud_cdc_n_connected(itf);
    }
}

static void diag_count_tx(uint32_t written, uint32_t dropped) {
    cdc_stats[DIAG_CDC_ITF].tx_bytes += written;
    cdc_stats[DIAG_CDC_ITF].tx_drop_bytes += dropped;
}

static void diag_write_text(const char *text, size_t len) {
    if (!tud_cdc_n_connected(DIAG_CDC_ITF) || len == 0) {
        return;
    }

    uint32_t available = tud_cdc_n_write_available(DIAG_CDC_ITF);
    if (available == 0) {
        diag_count_tx(0, (uint32_t)len);
        return;
    }

    uint32_t to_write = (len <= available) ? (uint32_t)len : available;
    uint32_t written = tud_cdc_n_write(DIAG_CDC_ITF, text, to_write);
    tud_cdc_n_write_flush(DIAG_CDC_ITF);
    diag_count_tx(written, (uint32_t)len - written);
}

static void emit_diagnostics_report(bool force) {
    refresh_cdc_connection_state();

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (!force && now_ms < next_diag_report_ms) {
        return;
    }

    next_diag_report_ms = now_ms + DIAG_REPORT_INTERVAL_MS;

    if (!cdc_stats[DIAG_CDC_ITF].connected) {
        return;
    }

    size_t used = 0;
    int len = snprintf(
        diag_report_buf + used, sizeof(diag_report_buf) - used,
        "\r\n[diag %lu] up=%lums usb=%u diag(dtr=%u rts=%u rx=%llu tx=%llu drop=%llu baud=%lu %u%s%s)\r\n",
        (unsigned long)diag_report_seq++,
        (unsigned long)now_ms,
        tud_mounted() ? 1u : 0u,
        cdc_stats[DIAG_CDC_ITF].dtr ? 1u : 0u,
        cdc_stats[DIAG_CDC_ITF].rts ? 1u : 0u,
        (unsigned long long)cdc_stats[DIAG_CDC_ITF].rx_bytes,
        (unsigned long long)cdc_stats[DIAG_CDC_ITF].tx_bytes,
        (unsigned long long)cdc_stats[DIAG_CDC_ITF].tx_drop_bytes,
        (unsigned long)cdc_stats[DIAG_CDC_ITF].line_coding.bit_rate,
        (unsigned)cdc_stats[DIAG_CDC_ITF].line_coding.data_bits,
        parity_to_str(cdc_stats[DIAG_CDC_ITF].line_coding.parity),
        stop_bits_to_str(cdc_stats[DIAG_CDC_ITF].line_coding.stop_bits)
    );
    if (len < 0) {
        return;
    }
    used += (size_t)len;

    for (uint8_t port = 0; port < BRIDGE_PORT_COUNT && used < sizeof(diag_report_buf); ++port) {
        uint64_t delta_usb_to_uart = port_stats[port].usb_to_uart_bytes - last_report[port].usb_to_uart_bytes;
        uint64_t delta_uart_to_usb = port_stats[port].uart_to_usb_bytes - last_report[port].uart_to_usb_bytes;
        uint64_t delta_uart_drop = port_stats[port].uart_to_usb_drop_bytes - last_report[port].uart_to_usb_drop_bytes;
        uint64_t delta_cdc_rx = cdc_stats[port].rx_bytes - last_report[port].cdc_rx_bytes;
        uint64_t delta_cdc_tx = cdc_stats[port].tx_bytes - last_report[port].cdc_tx_bytes;
        uint64_t delta_cdc_drop = cdc_stats[port].tx_drop_bytes - last_report[port].cdc_tx_drop_bytes;

        len = snprintf(
            diag_report_buf + used, sizeof(diag_report_buf) - used,
            "P%u %-7s c=%u dtr=%u rts=%u uart=%s pend=%u baud=%lu %u%s%s usb>uart=%llu(+%llu) uart>usb=%llu(+%llu) drop=%llu(+%llu) cdc_rx=%llu(+%llu) cdc_tx=%llu(+%llu) cdrop=%llu(+%llu)\r\n",
            port,
            port_names[port],
            cdc_stats[port].connected ? 1u : 0u,
            cdc_stats[port].dtr ? 1u : 0u,
            cdc_stats[port].rts ? 1u : 0u,
            port < 2 ? "HW" : "PIO",
            uart_rx_pending(port) ? 1u : 0u,
            (unsigned long)port_baud[port],
            (unsigned)cdc_stats[port].line_coding.data_bits,
            parity_to_str(cdc_stats[port].line_coding.parity),
            stop_bits_to_str(cdc_stats[port].line_coding.stop_bits),
            (unsigned long long)port_stats[port].usb_to_uart_bytes,
            (unsigned long long)delta_usb_to_uart,
            (unsigned long long)port_stats[port].uart_to_usb_bytes,
            (unsigned long long)delta_uart_to_usb,
            (unsigned long long)port_stats[port].uart_to_usb_drop_bytes,
            (unsigned long long)delta_uart_drop,
            (unsigned long long)cdc_stats[port].rx_bytes,
            (unsigned long long)delta_cdc_rx,
            (unsigned long long)cdc_stats[port].tx_bytes,
            (unsigned long long)delta_cdc_tx,
            (unsigned long long)cdc_stats[port].tx_drop_bytes,
            (unsigned long long)delta_cdc_drop
        );
        if (len < 0) {
            break;
        }
        if ((size_t)len >= sizeof(diag_report_buf) - used) {
            used = sizeof(diag_report_buf) - 1;
            break;
        }
        used += (size_t)len;

        last_report[port].usb_to_uart_bytes = port_stats[port].usb_to_uart_bytes;
        last_report[port].uart_to_usb_bytes = port_stats[port].uart_to_usb_bytes;
        last_report[port].uart_to_usb_drop_bytes = port_stats[port].uart_to_usb_drop_bytes;
        last_report[port].cdc_rx_bytes = cdc_stats[port].rx_bytes;
        last_report[port].cdc_tx_bytes = cdc_stats[port].tx_bytes;
        last_report[port].cdc_tx_drop_bytes = cdc_stats[port].tx_drop_bytes;
    }

    diag_write_text(diag_report_buf, used);
}

static void service_diag_cdc_rx() {
    if (!tud_cdc_n_available(DIAG_CDC_ITF)) {
        return;
    }

    uint32_t count = tud_cdc_n_read(DIAG_CDC_ITF, bridge_buf, sizeof(bridge_buf));
    cdc_stats[DIAG_CDC_ITF].rx_bytes += count;

    for (uint32_t i = 0; i < count; ++i) {
        if (bridge_buf[i] == '\r' || bridge_buf[i] == '\n' || bridge_buf[i] == '?') {
            emit_diagnostics_report(true);
            break;
        }
    }
}

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
    if (itf < TOTAL_CDC_COUNT) {
        cdc_stats[itf].line_coding = *p_line_coding;
    }

    if (itf < BRIDGE_PORT_COUNT) {
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

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    if (itf < TOTAL_CDC_COUNT) {
        cdc_stats[itf].dtr = dtr;
        cdc_stats[itf].rts = rts;
        cdc_stats[itf].connected = tud_cdc_n_connected(itf);
    }
}

} // extern "C"

// --- Bridge data transfer ---

// USB CDC -> Hardware UART
static void bridge_usb_to_hw_uart(uint8_t cdc_itf, uart_inst_t *uart) {
    if (!tud_cdc_n_available(cdc_itf)) return;

    uint32_t count = tud_cdc_n_read(cdc_itf, bridge_buf, sizeof(bridge_buf));
    cdc_stats[cdc_itf].rx_bytes += count;
    port_stats[cdc_itf].usb_to_uart_bytes += count;
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
    if (count > 0) {
        uint32_t written = tud_cdc_n_write(cdc_itf, bridge_buf, count);
        tud_cdc_n_write_flush(cdc_itf);
        cdc_stats[cdc_itf].tx_bytes += written;
        cdc_stats[cdc_itf].tx_drop_bytes += count - written;
        port_stats[cdc_itf].uart_to_usb_bytes += written;
        port_stats[cdc_itf].uart_to_usb_drop_bytes += count - written;
    }
}

// USB CDC -> PIO UART TX
static void bridge_usb_to_pio_uart(uint8_t cdc_itf, uint pio_idx) {
    if (!tud_cdc_n_available(cdc_itf)) return;

    uint32_t count = tud_cdc_n_read(cdc_itf, bridge_buf, sizeof(bridge_buf));
    cdc_stats[cdc_itf].rx_bytes += count;
    port_stats[cdc_itf].usb_to_uart_bytes += count;
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
    if (count > 0) {
        uint32_t written = tud_cdc_n_write(cdc_itf, bridge_buf, count);
        tud_cdc_n_write_flush(cdc_itf);
        cdc_stats[cdc_itf].tx_bytes += written;
        cdc_stats[cdc_itf].tx_drop_bytes += count - written;
        port_stats[cdc_itf].uart_to_usb_bytes += written;
        port_stats[cdc_itf].uart_to_usb_drop_bytes += count - written;
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
    next_diag_report_ms = to_ms_since_boot(get_absolute_time()) + DIAG_REPORT_INTERVAL_MS;

    while (true) {
        tud_task();
        refresh_cdc_connection_state();

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

        service_diag_cdc_rx();
        emit_diagnostics_report(false);
    }
}
