**Demo code for the HAOYU STM32F429IG w/ 5" HD TFT on system board.**

### Demo's

* **`gpio.cpp`** - GPIO access to on-board LEDs, I2C devices, and SPI flash
* **`ram.cpp`** - set up and access the 4 MB PSRAM and the 32 MB SDRAM
* **`lcd.cpp`** - set up and access the 800x480 LCD display
* **`font.cpp`** - generate 100x30 char text output on the LCD display

### On-board memory

Type   | Start      | Limit      | Total
-------|:----------:|:----------:|-----:
NOR    | 0x60000000 | 0x607FFFFF | 8 MB
PSRAM  | 0x6C000000 | 0x6C3FFFFF | 4 MB
NAND   | 0x80000000 | 0x87FFFFFF | 128 MB
SDRAM  | 0xC0000000 | 0xC0FFFFFF | 32 MB
SPI    | SPI2       | -          | 4 MB
I2C    | 0x50       | 0x57       | 2 KB

- on I2C bus H5+H4: 0x1A = WM8978 audio codec
- on I2C bus H8+H7: 0x38 = FT5x06 capacitive touch

- LEDs: B0, B1, C13 (on CPU board), A4 (LCD backlight)

### Sample "ram" output

    STM32F429x: ram @ 180 MHz
    4 MB PSRAM: (32b)
    000: 40414243 44454647 48494a4b 4c4d4e4f @ABC DEFG HIJK LMNO
    010: 50515253 54555657 58595a5b 5c5d5e5f PQRS TUVW XYZ[ \]^_
    +********************************************++++++++++++ 0 errors
    32 MB SDRAM: (32b)
    000: 60616263 64656667 68696a6b 6c6d6e6f `abc defg hijk lmno
    010: 70717273 74757677 78797a7b 7c7d7e7f pqrs tuvw xyz{ |}~.
    +********************************************++++++++++++ 0 errors
