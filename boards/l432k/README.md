### Example code for a Nucleo-32 L432KC board.

This reuses the test board and code from the `../g431k/` folder.
A slightly adjusted `platformio.ini` file specifies the pin differences.

See `../g431k/README.md` for further details.

```text
i2c: STM32L4x2 @ 80 MHz - 2024-12-02 22:24:57.996
	 i2c=B7:OH4,B6  spi=B5:H5,B4,B3,A11:HP  uart=A2:7,A15:3
	 led=A0  vcc=B1  bmp=A11  lcd=A12  sram=B0  sd=A7
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- 1E --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- 3C 3D -- --
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: 50 -- -- 53 -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- 77
```
