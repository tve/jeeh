# Baseline tests

Configured for a Nucleo-32 G431KB board.

Use any of these commands to verify that PlatformIO works properly:

    pio run -e blink     # blink the on-board LED with busy waiting
    pio run -e fast      # increase the clock from 16 to 160 MHz
    pio run -e serial    # send test output to USB via the ST-Link

Run all tests on the attached board with: **`pio test`**

- **smoke** - quick check that the test setup works
- **worker** - basic tests for `Event` and `Worker`
- **rtc** - built-in Real Time Clock test (using LSI, not LSE)
- **tick** - set up SysTick interrupt and test timer chain in `Ticker`
- **exti** - set up EXTI interrupts and test `ExtIrq` (needs PA9-PA10 jumper)
- **uart** - test variants of the UART driver (needs same PA9-PA10 jumper)
- **spi** - test variants of the SPI driver (needs SPI flash chip)
- **rng** - a quick check that the random number generator h/w works
- **adc** - verify that the DAC and ADC work (this needs PA4-PA7 jumper)

Serial output is 1,000,000 baud @ 16 MHz or 10,000,000 baud @ 160 MHz.

Test setup for external SPI flash and I2C FRAM:

![](board.jpg)

- SPI: MOSI=B5 MISO=B4 SCLK=B3 SSEL=A11 (W25Q16: 2 MB)
- I2C: SDA=B7 SCL=A15 (MB85RC256V: 32 KB)
