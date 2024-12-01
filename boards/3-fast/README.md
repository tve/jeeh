## Running the same code on different µC variants

- Fast clock, polled console @ 2 Mbaud, and tasks with periodic events.
- All board details are in "defs.h", using settings from "platformio.ini".

Build and upload for specific board, e.g. **`pio run -e g431k -t upload`**

Board      | CPU            | Command
-----------|----------------|------------------------------
Discovery  | STM32\<cpu\>xx | `pio run -e <cpu>d -t upload`
Nucleo-32  | STM32\<cpu\>Kx | `pio run -e <cpu>k -t upload`
Nucleo-64  | STM32\<cpu\>Rx | `pio run -e <cpu>r -t upload`
Nucleo-144 | STM32\<cpu\>Zx | `pio run -e <cpu>z -t upload`
