**Demo code for the HAOYU STM32F429IG w/ 5" HD TFT on system board.**

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
