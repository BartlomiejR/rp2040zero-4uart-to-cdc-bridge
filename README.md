# USB to 4-Port UART Bridge

RP2040-based USB to 4-UART bridge. Each UART appears as a separate `/dev/ttyACMx` on the host.

## Hardware

**Board:** Waveshare RP2040-Zero

| Port | Host Device | Type | TX | RX | Target |
|------|-------------|------|----|----|--------|
| 0 | ttyACM0 | HW UART0 | GP0 | GP1 | YDLidar TMini Plus |
| 1 | ttyACM1 | HW UART1 | GP4 | GP5 | DDSM115 |
| 2 | ttyACM2 | PIO UART | GP8 | GP9 | IMU (high speed) |
| 3 | ttyACM3 | PIO UART | GP12 | GP13 | Aux |

## Features

- 4 independent CDC ACM interfaces over a single USB connection
- Host-configurable baud rate per port (set automatically when opening the serial port)
- HW UARTs support configurable data bits, parity, and stop bits
- PIO UARTs fixed at 8N1, support high baud rates (tested up to several Mbaud)
- Default baud: 115200 on all ports

## Building

Requires the [Pico SDK](https://github.com/raspberrypi/pico-sdk) (tested with 2.2.0).

```
cd build
cmake ..
cmake --build . -j$(nproc)
```

Flash `build/uart_bridge.uf2` via the RP2040 USB bootloader (hold BOOT while plugging in).

## Identifying Ports

Each CDC interface has a descriptive USB string (visible via `udevadm info`):

```
UART0 - LiDAR
UART1 - DDSM115
UART2 - IMU
UART3 - Aux
```

To create stable symlinks, add a udev rule matching the interface number:

```
# /etc/udev/rules.d/99-uart-bridge.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="4004", ENV{ID_USB_INTERFACE_NUM}=="00", SYMLINK+="lidar"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="4004", ENV{ID_USB_INTERFACE_NUM}=="02", SYMLINK+="ddsm115"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="4004", ENV{ID_USB_INTERFACE_NUM}=="04", SYMLINK+="imu"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="4004", ENV{ID_USB_INTERFACE_NUM}=="06", SYMLINK+="aux"
```
