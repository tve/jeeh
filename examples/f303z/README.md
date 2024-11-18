### Example code for a Nucleo-144 F303ZE board.

1.44" LCD 128x128 w/ ST7735S

```text
LED     (tied to VCC)
SCK     SCLK    A5:5    D13
SDA     MOSI    A7:5    D11
A0      C/D     A6      D12
RESET   -       D15     D9
CS      NSEL    D14     D10
GND     -               GND
VCC     3.3-5V          3.3V
```

Results:

```text
lcd: STM32F303xE @ 72 MHz - 2024-11-18 23:09:28.777
init   1043 us
clear 31192 us
pixel    19 us
font1  8082 us (16 ch, 88 px)
font2  9491 us (16 ch, 112 px)
font3  4171 us (6 ch, 54 px)
```
