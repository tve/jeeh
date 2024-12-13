// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "f103rc"

//CG1 board leds
#define LED  "D2"

//CG[ board uart
#define UART_NAME UART5
constexpr uart::Config UART_CONF {
    "C12:U1,D2", UART5.ADDR, ena::UART5, 36,
};
//CG]
