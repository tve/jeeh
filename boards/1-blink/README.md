## Minimal LED blinker

Build and upload for specific board, e.g. **`pio run -e g431k -t upload`**

Board      | CPU            | Command
-----------|----------------|------------------------------
Discovery  | STM32\<cpu\>xx | `pio run -e <cpu>d -t upload`
Nucleo-32  | STM32\<cpu\>Kx | `pio run -e <cpu>k -t upload`
Nucleo-64  | STM32\<cpu\>Rx | `pio run -e <cpu>r -t upload`
Nucleo-144 | STM32\<cpu\>Zx | `pio run -e <cpu>z -t upload`

Other boards:

- Blue Pill: `f103bp` (via BMP)
- HyTiny: `f103hy` (via BMP)
- Black Pill: `f411bp`
