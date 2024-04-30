// Lines with "CG" control the code-generated parts of this file.

//CG[ board leds
#define LED  "B0"
#define LED1 "B0"
#define LED2 "B1"
#define LED3 "C13"
#define LED4 "A4"
//CG]

//CG[ board uart
#define UART_NAME  USART1
#define UART_PINS  "A9:7,A10"
#define UART_FREQ  90
#define UART_CONF  Irq::DMA2_Stream7,Irq::DMA2_Stream5,2-1,7-0,5-0,4,4
//CG]

constexpr Pin ledL (LED1), ledR (LED2), ledC (LED3), ledB (LED4);

void jeeh::fail(void const* a, char const* f, int n) {
    printf("\nfailed at %s:%d\nfailed caller: %p\n", f, n, a);
    while (true) {}
}

void jeeh::hardFaultHandler (uint32_t* sp) {
    printf("\nhard fault, sp = %p", sp);
    fail();
}

Uart uart ('U');

extern "C" int _write (int, char* ptr, int len) {
    Message m { 'U', 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

void initBoard () {
    hardFaulter = hardFaultHandler;
    fastClock();
    cycles::init();

    ledL.mode("P"); ledL = 1; // inverted logic
    ledR.mode("P"); ledR = 1; // inverted logic
    ledC.mode("P"); ledC = 1; // inverted logic
    ledB.mode("P");

    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });
}
