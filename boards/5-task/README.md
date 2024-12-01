## 5 - Run multiple tasks with periodic events

This code creates two tasks: the Ticker is a utility to manage software
timers and the Blinker toggles an LED every 500 ms (i.e. at 1 Hz). A periodic
ticker is set up to send events to the blinker, which then toggles its LED.

There is no additional configuration in `platformio.ini` since it can simply
reuse the one from `../1-blink/`. The output will have a precise 500 ms
repetition rate, because the time spent sending text over serial will not
affect the periodic events. All timing now uses SysTick hardware interrupts.

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
