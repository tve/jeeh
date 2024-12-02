### Example code for a Nucleo-32 G431KB board.

These examples are for use with this board:

![](board.jpg)

- SPI: BMP390, 32 KB SRAM, 128x128 TFT LCD, µSD card
- I2C: BMP390, 128x64 OLED, 128x32 OLED, IMU, SHT21, 32 KB FRAM

```text
i2c: STM32G431xx @ 160 MHz - 2024-12-02 20:46:01.019
	 i2c=B7:OH4,A15  spi=B5:H5,B4,B3,A11:HP  uart=A2:U7,A3
	 led=B8  vcc=B6  bmp=A11  lcd=A12  sram=B0  sd=A7
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- 1E --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- 3C 3D -- --
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: 50 -- -- 53 -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- 77
```

Hard-wired: A4 <=> A1 (DAC & ADC), P9 <=> P10 (USART1 RX & TX).

---

Another board is used for tests with an RFM69 radio module:

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
