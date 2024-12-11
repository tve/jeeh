# I2C + SPI tests

Sample output, see `main.cpp` for details:

```text
sync: STM32G431xx @ 160 MHz - 2024-12-11 09:59:12.097
	 i2c=B7:OH4,A15  spi=B5:H5,B4,B3  uart=A2:U7,A3
	 led=B8  vcc=B6  bmp=A11  lcd=A12  sram=B0  sd=A7
                                                            I2C - SCAN
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- 1E --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- 3C 3D -- --
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: 50 -- -- 53 -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- 77
        HMC5883 @ 0x1E: OK
    128x32 OLED @ 0x3C: OK
    128x64 OLED @ 0x3D: OK
          SHT21 @ 0x40: OK
     32 KB FRAM @ 0x50: OK
        ADXL345 @ 0x53: OK
        ITG3200 @ 0x68: OK
         BMP390 @ 0x77: OK
                                                            I2C - OLED
clear 1: 14019 µs
 oled 1:  6249 µs
clear 2:  7013 µs
 oled 2:  3125 µs
                                                            I2C - FRAM
EE EE ... EE EE
EE EE ... EE EE
EE EE ... EE EE
80 80 ... 81 81
81 81 ... EE EE
80 81 EE
                                                             I2C - IMU
hmc5883 compass: xyz =    110   -267    -20
hmc5883 compass: xyz =    109   -267    -25
hmc5883 compass: xyz =    114   -268    -15
adxl345 accel:   xyz =     32      0    240
adxl345 accel:   xyz =     33     -1    237
adxl345 accel:   xyz =     33      0    239
itg3200 gyro:    xyz =     -6     25      8
itg3200 gyro:    xyz =     -7     26      8
itg3200 gyro:    xyz =     -7     26      9
                                                           I2C - SHT21
T: 19.65 °C RH: 46.52 %
T: 19.66 °C RH: 46.52 %
T: 19.67 °C RH: 46.51 %
                                                          I2C - BMP390
raw T: 007D4700 P: 005C7100 => T: 21.326 °C P: 1032.065 hPa
raw T: 007D4800 P: 005C7100 => T: 21.331 °C P: 1032.076 hPa
raw T: 007D4A00 P: 005C7000 => T: 21.340 °C P: 1032.122 hPa
                                                       SPI - BMP390
raw T: 007D4F00 P: 005C7200 => T: 21.363 °C P: 1032.134 hPa
raw T: 007D5000 P: 005C7500 => T: 21.368 °C P: 1032.078 hPa
raw T: 007D5200 P: 005C7200 => T: 21.377 °C P: 1032.168 hPa
                                                             SPI - LCD
init      40 us
clear  32375 us
pixel     18 us
font1   7572 us (16 ch, 88 px)
font2   8963 us (16 ch, 112 px)
font3   3982 us (6 ch, 54 px)
                                                            SPI - SRAM
 000: 00000000 00000000 00006865 6c6c6f00 .... .... ..he llo.
 010: 00000000 776f726c 64000000 00000000 .... worl d... ....
 020: 00000000 00000000 00006865 6c6c6f2d .... .... ..he llo-
 030: 7370692d 776f726c 64000000 00000000 spi- worl d... ....
                                                                  DONE
```
