## 6 - Using async I/O for serial output

This is also an example to send large amounts of text over serial, but now the
CPU will be mostly idle (and sleeping in main's WFI loop). With async requests,
an additional reply event needs to be set up, which will be sent on completion.

The UART driver will start the transfer and immediately return to the caller.
The blink task (blinker) can run at the lowest priority because the send task
(streamer) no longer blocks while its serial output transfer is ongoing.

There's again no additional configuration in `platformio.ini` since it can reuse
the board and DMA configuration from `../1-blink/` + `../4-dma/`.

To build & upload for a specific board: **`pio run -e g431k -t upload`**

To see the output, use this cmd in a separate window: **`pio device monitor`**
(be sure to run it from this directory so it'll use the proper 2 Mbaud rate).

Board      | CPU            | Command
-----------|----------------|------------------------------
Discovery  | STM32\<cpu\>xx | `pio run -e <cpu>d -t upload`
Nucleo-32  | STM32\<cpu\>Kx | `pio run -e <cpu>k -t upload`
Nucleo-64  | STM32\<cpu\>Rx | `pio run -e <cpu>r -t upload`
Nucleo-144 | STM32\<cpu\>Zx | `pio run -e <cpu>z -t upload`

See `platformio.ini` to see which boards have been defined so far.
