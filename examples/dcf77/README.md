## DCF77 receiver using STM32L496 Discovery

PMOD connector, lower row:

Pin | Signal | PMOD | Notes
----|--------|------|------
 6  | VCC    |      | nearest to ST-Link USB
 5  | GND    |      |
 4  | N/C    |      |
 3  | N/C    |      |
 2  | PB2    | # 12 | data, needs pull-up
 1  | PH2    | # 11 | power, 0 = on

### Implementations:

**blink.cpp** - Blink both on-board LEDs and show a greeting on the serial port.
This is just to verify that both the build and the board work (use: `pio run -e
blink -t upload`). The serial output goes to the ST-Link at 2 Mbaud (use: `pio
device monitor`).

**signal.cpp** - Mirror the DCF77 input signal to the LED, to verify that the
radio module is properly connected and is receiving a signal.

**cycle.cpp** - Dumb polled-mode DCF77 decoder, using just the DWT cycle counter
for all timing and delays (with a time resolution is 1000 Hz). Does not use any
timers or interrupts.

**rtc.cpp** - Dumb polled-mode DCF77 decoder, using just the RTC clock for all
timing and delays (with a time resolution is 256 Hz). Does not use any timers or
interrupts.

**push.cpp** - Turn the decoder logic inside out: sample the DCF77 pin
periodically and feed it to a decoder object. Does not use any timers or
interrupts.
