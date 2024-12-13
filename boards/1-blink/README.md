## 1 - Minimal LED blinker

This example uses a minimal subset of the JeeH library to blink an on-board LED.
The board-specific pin is set in `platformio.ini` and copied to `defs.h`.

To build & upload for a specific board: **`pio run -e g431k -t upload`**

Board      | CPU            | Command
-----------|----------------|------------------------------
Discovery  | STM32\<cpu\>xx | `pio run -e <cpu>d -t upload`
Nucleo-32  | STM32\<cpu\>Kx | `pio run -e <cpu>k -t upload`
Nucleo-64  | STM32\<cpu\>Rx | `pio run -e <cpu>r -t upload`
Nucleo-144 | STM32\<cpu\>Zx | `pio run -e <cpu>z -t upload`

Other boards:

- Blue Pill: `f103bp` (via BMP)
- HyTiny: `f103hy` (via BMP)
- Olimexino F1: `f103o`
- No-name F103RC: `f103rc`
- Olimexino F3: `f303o`
- Black Pill: `f411bp`

See `platformio.ini` to see which boards have been defined so far.
