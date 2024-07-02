**Readout of a BMP390 pressure sensor via either I2C or SPI.**

Note: `L` = left 15-pin connector (CN3), `R` = right 15-pin connector (CN4)

Pin | GPIO | BMP390 | Arduino | Use
----|------|:------:|:-------:|-------
L7  | PB7  | SDI    | D4      | SDA
L8  | PB6  | SCK    | D5      | SCL
L9  | PB1  | VCC    | D6      | POWER
L12 | PA8  | INT    | D9      | IRQ
L13 | PA11 | CS     | D10     | NSEL
L14 | PB5  | SDI    | D11     | MOSI
L15 | PB4  | SDO    | D12     | MISO
R2  | -    | GND    | GND     | GND
R3  | NRST | -      | RESET   | BUTTON
R15 | PB3  | SCK    | D13     | SCLK

![](board.jpg)
