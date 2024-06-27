// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "send-call"

//CG1 board leds
#define LED  "B8"

constexpr Pin led (LED);

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:UH7,A3"
#define UART_FREQ  170
#define UART_CONF  Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,1-1,2-1,27,26
//CG]

#define UART_TYPE  USART2.ADDR,DMA1.ADDR,1-1,2-1
#define UART_OCONF  { ena::USART2,170,Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,27,26 }


//CG1 board mode
#define MODE_CALL 1

#if MODE_POLL
uart::Poll<UART_NAME.ADDR> console (ena::UART_NAME, UART_FREQ);
#elif MODE_SYNC
uart::Sync<UART_TYPE> console (UART_OCONF);
#elif MODE_CALL
uart::Call<UART_TYPE> console (UART_OCONF);
#endif

//CG[ board uart2
#define UART2_NAME  USART1
#define UART2_PINS  "A9:7,A10"
#define UART2_FREQ  170
#define UART2_CONF  Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,1-1,2-1,25,24
//CG]

void initBoard () {
    fastClock();
    led.mode("P");
    cycles::init();
    rtc::init(false);

#if 0
    ouart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                     UART_FREQ, Irq::UART_NAME, UART_CONF });
#endif

    console.init(UART_PINS, 115'200);
    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}

extern "C" int _write (int, char* ptr, int len) {
    console.transfer(1, (uint8_t*) ptr, len);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}
