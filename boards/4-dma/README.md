## 4 - High-speed serial text using DMA + WFE

This example only differs from the previous one in that it uses DMA for UART
transfers and WFE to sleep until transfer completion. The `platformio.ini` file
needs to contain more details on how to configure the DMA hardware.

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
