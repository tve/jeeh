## 2 - Minimal serial output

Using direct register access, this code sets up a 115200 baud serial connection
to the ST-Link and emits periodic timestamps based on the CPU cycle counter
(except on M0+).  Also blinks the on-board LED.  The board-specific LED and UART
details are defined in `platformio.ini` and copied to `defs.h`.

To build & upload for a specific board: **`pio run -e g431k -t upload`**

To see the output, use this cmd in a separate window: **`pio device monitor`**
(be sure to run it from this directory so it'll use the proper 115200 bd rate)

Board      | CPU            | Command
-----------|----------------|------------------------------
Discovery  | STM32\<cpu\>xx | `pio run -e <cpu>d -t upload`
Nucleo-32  | STM32\<cpu\>Kx | `pio run -e <cpu>k -t upload`
Nucleo-64  | STM32\<cpu\>Rx | `pio run -e <cpu>r -t upload`
Nucleo-144 | STM32\<cpu\>Zx | `pio run -e <cpu>z -t upload`

See `platformio.ini` to see which boards have been defined so far.
