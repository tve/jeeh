# Example apps

This is a sequence of "bring-up" examples for a variety of boards:

* `1-blink/` - minimal LED blink demo
* `2-serial/` - minimal serial output demo
* `3-fast/` - fast CPU clock, polled serial output @ 2 Mbaud
* `4-dma/` - switch from polled serial output to DMA + WFE
* `5-task/` - using multiple tasks with periodic event triggers
* `6-async/` - asynchronous full speed serial + background LED blink

## Boards

In the list below, each example can be built and run independently, e.g.

    cd g431k
    pio run -e i2c -t upload
    pio device monitor          # to view serial output

**`F303K8` - Nucleo-32 @ 72 MHz**

* This is a plug-in variation for use with the **G431KB** test board below.

**`F303RC` - Olimexino w/ ST-Link @ 72 MHz**

* `f303o/dcf77` - decode DCF77 time pulses using 1st radio module
* `f303o/msf60` - decode MSF60 time pulses using 2nd radio module
* `f303o/gps` - show messages coming from attached GPS module
* `f303o/stream` - capture DCF77 at 1 kHz and stream as 16-bit ints
* `f303o/app` - demo app to explore time-related functionality

**`F413ZH` - Nucleo-144 @ 100 MHz**

* `f413z/uarts` - send data through multiple uarts using async driver

**`F429ZI` - Haoyu w/ 5" TFT @ 180 MHz**

* `f429h/gpio` - check presence of I2C, SPI, and LCD peripherals
* `f429h/lcd` - draw a transparency test pattern on the 800x480 LCD
* `f429h/font` - write 8x16 font as a screen full of 100x30 chars
* `f429h/life` - run Conway's "Game of Life" on a 400x240 grid
* `f429h/ram` - test on-board 4 MB PSRAM and 32 MB SDRAM memory

**`F446ZE` - Nucleo-144 @ 180 MHz**

* `f446z/gps` - show messages coming from attached GPS module
* `f446z/lcd` - display text and graphics on a 128x128 LCD over SPI

**`G431KB` - Nucleo-32 @ 160 MHz**

* `g431k/serial` - send text to serial port @ 10 Mbaud
* `g431k/i2c` - scan for attached devices on the I2C bus
* `g431k/dog` - kick the watchdog less and less, until it resets
* `g431k/oled` - show a test pattern on 128x64 or 128x32 I2C OLEDs
* `g431k/imu` - read out the values of a 9-DOF I2C IMU sensor
* `g431k/pulses` - generate pulses to check the logic-analyser hookup
* `g431k/bmp390i` - read out the BMP390 sensor using its I2C pins
* `g431k/bmp390s` - read out the BMP390 sensor using its SPI pins
* `g431k/fram` - test the I2C bus with attached 32 KB FRAM
* `g431k/sram` - test the SPI bus with attached 32 KB SRAM
* `g431k/sht21` - read temperature and humidity from SHT21 sensor
* `g431k/lcd` - display text and graphics on a 128x128 LCD over SPI
* `g431k/sdspi` - connect to the µSD card socket via SPI
* `g431k/shell` - explore the use of a command-line shell

**`G474RE` - Nucleo-64 @ 160 MHz**

* `g474r/lcd` - graphics and text on a 480x320 LCD shield
* `g474r/sdspi` - connect to the µSD card on the LCD shield

**`L432KC` - Nucleo-32 @ 80 MHz**

* This is a plug-in variation for use with the **G431KB** test board above.

## Test board

There's a `test/` area which reuses some of the `g431/` code to run tests on
all I2C and SPI devices attached to the test board. The tests can run in
different modes, one for each driver variant (gpio, poll, etc):

```
pio run -e gpio -t upload
pio run -e poll -t upload
[etc...]
```

As before, use `pio device monitor` to see the test results.
