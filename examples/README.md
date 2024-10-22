# Example apps

Each application can be built and run independently, e.g.

    cd l412k
    pio run -e blink -t upload
    pio device monitor          # to view serial output

### Nucleo-32 w/ `G431KB`

* `g431k/blink`: blink on-board LED
* `g431k/fast`: blink with system clock at 160 MHz
* `g431k/serial`: send text to srial port @ 10 Mbaud
* `g431k/i2c`: scan for attached devices on the I2C bus
* `g431k/dog`: kick the watchdog less and less, until it resets
* `g431k/oled`: show a test pattern on 128x64 or 128x32 I2C OLEDs
* `g431k/imu`: read out the values of a 9-DOF I2C IMU sensor
* `g431k/pulses`: generate some pulses to check the logic-analyser hookup

### Nucleo-64 w/ `G431RE`

* `g431r/blink` - blink on-board LED
* `g431r/lcd` - graphics and text on a 480x320 LCD shield
* `g431r/sdspi` - connect to the µSD card on the LCD shield

### Nucleo-32 w/ `L412KB`

* `l412k/blink` - blink on-board LED and send some text to serial
* `l412k/i2c` - scan for attached devices on the I2C bus
* `l412k/spi` - read out the BMP390 sensor using its SPI pins

### Nucleo-64 w/ `L412RB_P`

* `l412r/blink` - blink on-board LED and send some text to serial
* `l412r/dcf77` - decode DCF77 time pulses using attached radio module

### Nucleo-144 w/ `F413ZH`

* `f413z/blink` - blink on-board LED and send some text to serial
