## DCF77 receiver using STM32L412RB Nucleo-64

Connector on DCF module:

Pin | Signal | Notes
----|--------|------
 1  | VCC    | near corner
 2  | GND    |
 3  | PC0    | data, needs pull-up
 4  | PC1    | power, 0 = on

### Implementations:

**blink.cpp** - Blink both on-board LEDs and show a greeting on the serial port.
This is just to verify that both the build and the board work (use: `pio run -e
blink -t upload`). The serial output goes to the ST-Link at 1 Mbaud (use: `pio
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

**convol.cpp** - This is a completely different decoder, based on convolution.
It was ported to JeeH from `stm32x/nucleo-l432/src/dcf77-conv.cpp`.

**lptimer.cpp** - Again the convolution decoder, but triggered from the
low-power timer (also at 256 Hz).

**systick.cpp** - Now the decoder is called directly from the `SysTick_Handler`
exception code, i.e. in handler mode. In preparation of workers & triggers ...
