# Baseline tests

Configured for a Nucleo-32 G431KB board.

 Use any of these commands to verify that PlatformIO has been properly set up:

    pio run -t upload -e blink     # blink the on-board LED with busy waiting
    pio run -t upload -e fast      # increase the clock from 16 to 170 MHz
    pio run -t upload -e serial    # send test output to USB via the ST-Link

Run all tests on the attached board with: **`pio test`**

The serial baudrate is 1,000,000 baud @ 16 MHz or 10,625,000 baud @ 170 MHz.
