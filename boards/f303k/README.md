### Example code for a Nucleo-32 F303K8 board.

This reuses the test board and code from the `../g431k/` folder.
A slightly adjusted `platformio.ini` file specifies the pin differences.

See `../g431k/README.md` for further details.

```text
i2c: STM32F30x @ 72 MHz - 2024-12-02 23:11:52.414
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
