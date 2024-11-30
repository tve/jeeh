// Lines with "CG" control the code-generated parts of this file.

//CG1 board leds
#define LED  "B3"

//CG2 board serout
#define SEROUT_DEV USART2
#define SEROUT_PIN "A2:7"

Pin led (LED, "P");
