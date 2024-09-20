# Baseline tests

Configured for a Nucleo-32 G431KB board.

Use any of these commands to verify that PlatformIO works properly:

    pio run -e blink     # blink the on-board LED with busy waiting
    pio run -e fast      # increase the clock from 16 to 160 MHz
    pio run -e serial    # send test output to USB via the ST-Link

Run all tests on the attached board with: **`pio test`**

The serial rate is 1,000,000 baud @ 16 MHz or 10,000,000 baud @ 160 MHz.
