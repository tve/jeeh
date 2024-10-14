**Hooking up a logic analyzer to a Nucleo-32 G431KB board.**

Connections on CN3 (right-side, USB on top):

Pin |  µC  |  Wire   | Arduino | Use
---:|------|---------|---------|----------
  2 | GND  | white   |  GND    | GND
  3 | NRST | gray    |  NRST   | RESET
  5 | PA2  | purple  |  A7     | USART2 TX
  6 | PA7  | blue    |  A6     | SPI1 MOSI
  8 | PA5  | green   |  A4     | SPI1 SCLK
  9 | PA4  | yellow  |  A3     | SPI1 NSEL
 11 | PA1  | orange  |  A1     | -
 12 | PA0  | red     |  A0     | -
 15 | PB3  | brown   |  D13    | SWO
