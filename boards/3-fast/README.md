## 3 - High-speed serial text in polled mode

This example switches to a fast CPU clock and raises the serial rate to 2 Mbaud.
It then sends continuous output of text lines, varying in length. This uses the
polled UART driver, i.e. the CPU remains busy while waiting for TX to complete.
The on-board LED is on for the first 64 of every 4096 lines of output. Due to
the very high data rate, it blinks at approx 1 Hz.

The `platformio.ini` contains board-specific details, including the selected
UART peripheral and its maximum AHB/APB bus rate in MHz. Some boards need
additional settings to properly set the system clock to its maximum frequency.

To build & upload for a specific board: **`pio run -e g431k -t upload`**

To see the output, use this cmd in a separate window: **`pio device monitor`**
(be sure to run it from this directory so it'll use the proper 2 Mbaud rate)

Board      | CPU            | Command
-----------|----------------|------------------------------
Discovery  | STM32\<cpu\>xx | `pio run -e <cpu>d -t upload`
Nucleo-32  | STM32\<cpu\>Kx | `pio run -e <cpu>k -t upload`
Nucleo-64  | STM32\<cpu\>Rx | `pio run -e <cpu>r -t upload`
Nucleo-144 | STM32\<cpu\>Zx | `pio run -e <cpu>z -t upload`

See `platformio.ini` to see which boards have been defined so far.
