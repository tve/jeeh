# Example apps

Each application can be built and run independently, e.g.

    cd l412k
    pio run -e blink -t upload
    pio device monitor          # to view serial output

**`nucleo_f103rb` @ 72 MHz**

* `f103r/blink` - blink on-board LED and send some text to serial
* `f103r/tick`: blink on-board LED via periodic timer and worker

**`nucleo_f303k8` @ 72 MHz**

* `f303k/blink` - blink on-board LED and send some text to serial
* `f303k/tick`: blink on-board LED via periodic timer and worker

**`nucleo_f413zh` @ 100 MHz**

* `f413z/blink` - blink on-board LED and send some text to serial
* `f413z/uarts` - send data through multiple uarts using async driver

**`nucleo_g431kb` @ 160 MHz**

* `g431k/blink`: minimal blink of on-board LED
* `g431k/fast`: blink with system clock at 160 MHz
* `g431k/serial`: send text to srial port @ 10 Mbaud
* `g431k/i2c`: scan for attached devices on the I2C bus
* `g431k/dog`: kick the watchdog less and less, until it resets
* `g431k/oled`: show a test pattern on 128x64 or 128x32 I2C OLEDs
* `g431k/imu`: read out the values of a 9-DOF I2C IMU sensor
* `g431k/pulses`: generate pulses to check the logic-analyser hookup
* `g431k/tick`: blink on-board LED via periodic timer and worker

**`nucleo_g431rb` @ 160 MHz**

* `g431r/blink` - blink on-board LED and send some text to serial
* `g431r/lcd` - graphics and text on a 480x320 LCD shield
* `g431r/sdspi` - connect to the µSD card on the LCD shield

**`nucleo_l031k6` @ 32 MHz**

* `l031k/blink` - blink on-board LED and send some text to serial
* `l031k/tick`: blink on-board LED via periodic timer and worker

**`nucleo_l412kb` @ 80 MHz**

* `l412k/blink` - blink on-board LED and send some text to serial
* `l412k/i2c` - scan for attached devices on the I2C bus
* `l412k/spi` - read out the BMP390 sensor using its SPI pins
* `l412k/tick`: blink on-board LED via periodic timer and worker

**`nucleo_l412rb_p` @ 80 MHz**

* `l412r/blink` - blink on-board LED and send some text to serial
* `l412r/dcf77` - decode DCF77 time pulses using attached radio module
