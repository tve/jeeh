### Example code for a Nucleo-32 G431KB board.

Use any of these commands to verify that PlatformIO works properly:

    pio run -e blink     # blink the on-board LED with busy waiting
    pio run -e fast      # increase the clock from 16 to 160 MHz
    pio run -e i2c       # list any devices connected to I2C via PB7+PA15


This board is used for tests with an RFM69 radio module:

![](rfm69.jpg)

Header | Nucleo | STM32 | RFM69 | Notes  |
------:|:------:|-------|-------|--------|
`3-01` | D1     | PA9   |       |        |
`3-02` | D0     | PA10  |       |        |
`3-03` | RESET  | NRST  |       |        |
`3-04` | GND    | -     |       |        |
`3-05` | D2     | PA12  |       |        |
`3-06` | D3     | PB0   |       |        |
`3-07` | D4     | PB7   |       |        |
`3-08` | D5     | PA15  | DIO5  |        |
`3-09` | D6     | PB6   | RESET |        |
`3-10` | D7     | PF0   |       | OSC32  |
`3-11` | D8     | PF1   |       | OSC32  |
`3-12` | D9     | PA8   | DIO3  |        |
`3-13` | D10    | PA11  | NSS   |        |
`3-14` | D11    | PB5   | MOSI  |        |
`3-15` | D12    | PB4   | MISO  |        |
`4-01` | VIN    | -     |       |        |
`4-02` | GND    | -     |       |        |
`4-03` | RESET  | NRST  |       |        |
`4-04` | +5V    | -     |       |        |
`4-05` | A7     | PA2   |       | VCP TX |
`4-06` | A6     | PA7   |       |        |
`4-07` | A5     | PA6   |       |        |
`4-08` | A4     | PA5   |       |        |
`4-09` | A3     | PA4   |       |        |
`4-10` | A2     | PA3   | DIO2  | VCP RX?|
`4-11` | A1     | PA1   | DIO1  |        |
`4-12` | A0     | PA0   | DIO0  |        |
`4-13` | AREF   | -     |       |        |
`4-14` | +3V3   | -     |       |        |
`4-15` | D13    | PB3   | SCK   | LED    |
