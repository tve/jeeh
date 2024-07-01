// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "send-poll"

//CG1 board leds
#define LED  "A5"

constexpr Pin led      (LED); // redundant, same as blue

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:7,A3"
#define UART_FREQ  170
#define UART_CONF  Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,1-1,2-1,27,26
//CG]

inline Uart console ('U');

//CG[ board uart_lb
#define UART_LB_NAME  USART1
#define UART_LB_PINS  "C4:7,C5"
#define UART_LB_FREQ  170
#define UART_LB_CONF  Irq::DMA1_CH3,Irq::DMA1_CH4,1-1,3-1,4-1,25,24
//CG]

inline Uart loopback ('L');

//CG[ board uart_lp
#define UART_LP_NAME  LPUART1
#define UART_LP_PINS  "C1:8,C0"
#define UART_LP_FREQ  170
#define UART_LP_CONF  Irq::DMA1_CH5,Irq::DMA1_CH6,1-1,5-1,6-1,35,34
//CG]

inline Uart lowpower ('P');

//CG1 board mode
#define MODE_POLL 1

void initBoard () {
    fastClock();
    led.mode("P");
    cycles::init();

    console.init(UART_PINS, 115'200,
                { UART_NAME.ADDR, ena::UART_NAME,
                  UART_FREQ, Irq::UART_NAME, UART_CONF });
    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);

    loopback.init(UART_LB_PINS, 57'600,
                { UART_LB_NAME.ADDR, ena::UART_LB_NAME,
                  UART_LB_FREQ, Irq::UART_LB_NAME, UART_LB_CONF });

    // TODO this baud rate is incorrect, needs a 256x adjustment somewhere ...
    lowpower.init(UART_LP_PINS, 57'600,
                { UART_LP_NAME.ADDR, ena::UART_LP_NAME,
                  UART_LP_FREQ, Irq::UART_LP_NAME, UART_LP_CONF });
}

extern "C" int _write (int, char* ptr, int len) {
    Message m { console.dId, 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}
