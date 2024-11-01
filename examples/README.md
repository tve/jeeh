# Example apps

Each application can be built and run independently, e.g.

    cd g431k
    pio run -e tick -t upload
    pio device monitor          # to view serial output

**`f103c8` - Blue Pill w/ BMP @ 72 MHz**

* `f103b/blink` - blink on-board LED and send some text to serial
* `f103b/tick` - blink on-board LED via periodic timer and worker

**`f103rb` - Nucleo-64 @ 72 MHz**

* `f103r/blink` - blink on-board LED and send some text to serial
* `f103r/tick` - blink on-board LED via periodic timer and worker

**`f303k8` - Nucleo-32 @ 72 MHz**

* `f303k/blink` - blink on-board LED and send some text to serial
* `f303k/tick` - blink on-board LED via periodic timer and worker

**`f303rc` - Olimexino w/ ST-Link @ 72 MHz**

* `f303o/blink` - blink on-board LED and send some text to serial
* `f303o/tick` - blink on-board LED via periodic timer and worker
* `f303o/dcf77` - decode DCF77 time pulses using 1st radio module
* `f303o/msf60` - decode MSF60 time pulses using 2nd radio module
* `f303o/gps` - show messages coming from attached GPS module
* `f303o/stream` - capture DCF77 at 1 kHz and strean as 16-bit ints

**`f413zh` - Nucleo-144 @ 100 MHz**

* `f413z/blink` - blink on-board LED and send some text to serial
* `f413z/uarts` - send data through multiple uarts using async driver
* `f413z/tick` - blink on-board LED via periodic timer and worker

**`f429zi` - Haoyu w/ 5" TFT @ 180 MHz**

* `f429h/gpio` - check presence of I2C, SPI, and LCD peripherals
* `f429h/lcd` - draw a transparency test pattern on the 800x480 LCD
* `f429h/font` - write 8x16 font as a screen full of 100x30 chars
* `f429h/life` - run Conway's "Game of Life" on a 400x240 grid
* `f429h/ram` - test on-board 4 MB PSRAM and 32 MB SDRAM memory

**`f746zg` - Nucleo-144 @ 216 MHz**

* `f746z/blink` - blink on-board LED and send some text to serial
* `f746z/tick` - blink on-board LED via periodic timer and worker

**`g031j6` - Discovery @ 64 MHz**

* `g031d/blink` - blink on-board LED and send some text to serial
* `g031d/tick` - blink on-board LED via periodic timer and worker

**`g431kb` - Nucleo-32 @ 160 MHz**

* `g431k/blink` - minimal blink of on-board LED
* `g431k/fast` - blink with system clock at 160 MHz
* `g431k/serial` - send text to serial port @ 10 Mbaud
* `g431k/i2c` - scan for attached devices on the I2C bus
* `g431k/dog` - kick the watchdog less and less, until it resets
* `g431k/oled` - show a test pattern on 128x64 or 128x32 I2C OLEDs
* `g431k/imu` - read out the values of a 9-DOF I2C IMU sensor
* `g431k/pulses` - generate pulses to check the logic-analyser hookup
* `g431k/tick` - blink on-board LED via periodic timer and worker

**`g431rb` - Nucleo-64 @ 160 MHz**

* `g431r/blink` - blink on-board LED and send some text to serial
* `g431r/lcd` - graphics and text on a 480x320 LCD shield
* `g431r/sdspi` - connect to the µSD card on the LCD shield

**`h743zi` - Nucleo-144 @ 480 MHz**

* `h743z/blink` - blink on-board LED and send some text to serial
* `h743z/tick` - blink on-board LED via periodic timer and worker

**`l031k6` - Nucleo-32 @ 32 MHz**

* `l031k/blink` - blink on-board LED and send some text to serial
* `l031k/tick` - blink on-board LED via periodic timer and worker

**`l412kb` - Nucleo-32 @ 80 MHz**

* `l412k/blink` - blink on-board LED and send some text to serial
* `l412k/i2c` - scan for attached devices on the I2C bus
* `l412k/spi` - read out the BMP390 sensor using its SPI pins
* `l412k/tick` - blink on-board LED via periodic timer and worker
