# USB to Multi-Port UART Bridge for RP2040

This project turns an RP2040 board into a compact USB-to-UART bridge with four independent serial ports and one extra USB CDC port for diagnostics.

It was built for a specific robotics setup, but the design is generic enough to reuse in other projects by changing the pin mapping, labels, and default baud rates.

## What It Does

- Exposes 4 bridged UART ports over a single USB connection
- Exposes 1 additional USB CDC port for bridge diagnostics
- Supports 2 hardware UARTs and 2 PIO-based UARTs
- Applies host-selected baud rates to the bridged UART ports
- Reports traffic counters and link state on the diagnostics port

The current target board is:

- Waveshare RP2040-Zero

## Port Map

| Bridge Port | USB CDC Label | UART Type | TX Pin | RX Pin | Default Baud | Intended Target |
|-------------|---------------|-----------|--------|--------|--------------|-----------------|
| 0 | `UART0 - LiDAR` | Hardware UART0 | GP0 | GP1 | 230400 | YDLidar TMini Plus |
| 1 | `UART1 - DDSM115` | Hardware UART1 | GP4 | GP5 | 115200 | DDSM115 |
| 2 | `UART2 - IMU` | PIO UART | GP8 | GP9 | 230400 | IMU |
| 3 | `UART3 - Aux` | PIO UART | GP12 | GP13 | 115200 | Auxiliary device |
| 4 | `Bridge Diagnostics` | USB CDC only | - | - | 115200 | Status and debugging |

## Notes About Baud Rate Control

The bridge follows USB CDC line-coding requests from the host. In practice, that means opening a port on the host and setting a baud rate will reconfigure the matching UART on the RP2040.

This is useful when the host is the source of truth, but it also means host-side probing tools or services can change UART baud rates unexpectedly. On Linux SBCs, services such as `ModemManager` or `serial-getty` may touch new serial devices and leave them configured at `9600`.

If you need fixed baud rates for some ports, update the firmware accordingly instead of relying on host configuration.

## Diagnostics Port

The fifth CDC interface is named:

```text
Bridge Diagnostics
```

It prints a periodic status snapshot with:

- USB connection state
- CDC DTR/RTS state
- Active baud and frame format
- Bytes moved from USB to UART
- Bytes moved from UART to USB
- Dropped bytes on the USB TX side
- Whether UART RX data is currently pending

You can trigger an immediate report by sending:

- `Enter`
- `?`

Example diagnostics output:

```text
[diag 45] up=131003ms usb=1 diag(dtr=1 rts=1 rx=1 tx=30385 drop=0 baud=115200 8N1)
P0 LiDAR   c=0 dtr=0 rts=0 uart=HW pend=0 baud=230400 8N1 usb>uart=0(+0) uart>usb=1(+0) drop=0(+0) cdc_rx=0(+0) cdc_tx=1(+0) cdrop=0(+0)
P1 DDSM115 c=0 dtr=0 rts=0 uart=HW pend=0 baud=115200 8N1 usb>uart=0(+0) uart>usb=198800(+1520) drop=0(+0) cdc_rx=0(+0) cdc_tx=198800(+1520) cdrop=0(+0)
P2 IMU     c=0 dtr=0 rts=0 uart=PIO pend=1 baud=230400 8N1 usb>uart=0(+0) uart>usb=2950364(+22533) drop=0(+0) cdc_rx=0(+0) cdc_tx=2950364(+22533) cdrop=0(+0)
P3 Aux     c=0 dtr=0 rts=0 uart=PIO pend=0 baud=115200 8N1 usb>uart=0(+0) uart>usb=0(+0) drop=0(+0) cdc_rx=0(+0) cdc_tx=0(+0) cdrop=0(+0)
```

## Build

This project uses the Raspberry Pi Pico SDK.

Tested with:

- Pico SDK `2.2.0`

Build steps:

```sh
mkdir -p build
cd build
cmake ..
cmake --build .
```

The firmware image will be generated as:

- `build/uart_bridge.uf2`

## Flash

### Using the USB Bootloader

1. Hold the `BOOT` button while plugging in the RP2040 board.
2. The board should appear as a USB mass-storage device.
3. Copy `build/uart_bridge.uf2` to it.
4. The board will reboot automatically into the new firmware.

### Using the Raspberry Pi Pico VS Code Extension

`Run Project (USB)` is suitable for normal persistent flashing.

## Host-Side Port Identification

The OS-assigned device names can change between boots, so it is better to identify ports by their USB interface string or interface number.

USB CDC labels exposed by this firmware:

```text
UART0 - LiDAR
UART1 - DDSM115
UART2 - IMU
UART3 - Aux
Bridge Diagnostics
```

### Linux

You can inspect the interfaces with:

```sh
udevadm info -a -n /dev/ttyACM0
```

Example `udev` rules for stable symlinks:

```udev
# /etc/udev/rules.d/99-uart-bridge.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="4004", ENV{ID_USB_INTERFACE_NUM}=="00", SYMLINK+="lidar"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="4004", ENV{ID_USB_INTERFACE_NUM}=="02", SYMLINK+="ddsm115"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="4004", ENV{ID_USB_INTERFACE_NUM}=="04", SYMLINK+="imu"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="4004", ENV{ID_USB_INTERFACE_NUM}=="06", SYMLINK+="aux"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="4004", ENV{ID_USB_INTERFACE_NUM}=="08", SYMLINK+="bridge_diag"
```

### macOS

List the serial devices:

```sh
ls /dev/cu.usbmodem*
```

To map a device node to the diagnostics interface:

```sh
ioreg -l -w 0 | rg -n -C 8 "Bridge Diagnostics|usbmodem"
```

Open it with:

```sh
screen /dev/cu.usbmodemXXXX 115200
```

or
```sh
python3 -m serial.tools.miniterm /dev/cu.usbmodemXXXXX 115200
```

## Firmware Behavior Summary

- Ports `0` and `1` use hardware UARTs
- Ports `2` and `3` use PIO UARTs
- Hardware UART ports apply host-selected data bits, parity, and stop bits
- PIO UART ports currently operate as `8N1`
- Diagnostics are reported on a separate fifth CDC interface

## Adapting This Project

The main places to customize are:

- `uart_bridge.cpp` for pin mapping, default baud rates, and bridge behavior
- `usb_descriptors.c` for USB port names and interface layout
- `tusb_config.h` for TinyUSB CDC configuration

## License

Add a license file before publishing if you plan to make the repository public.
