// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "uart"
//CG3 board leds
#define LED  "J5"
#define LED1 "J5"
#define LED2 "J13"

constexpr Pin led (LED);
constexpr Pin led1 (LED1);
constexpr Pin led2 (LED2);

//CG[ board uart
#define UART_NAME  USART1
#define UART_PINS  "A9:7,A10"
#define UART_FREQ  108
#define UART_CONF  Irq::DMA2_Stream7,Irq::DMA2_Stream5,2-1,7-0,5-0,4,4
//CG]

Uart console ('U');

void initFmcPins () {
    RCC(ena::FMC, 1) = 1;
    Pin::config("D0:UH12,D1,D8,D9,D10,D14,D15,"
                "E0,E1,E7,E8,E9,E10,E11,E12,E13,E14,E15,"
                "F0,F1,F2,F3,F4,F5,F11,F12,F13,F14,F15,"
                "G0,G1,G2,G4,G5,G8,G15,"
                "H2,H3,H5,H8,H9,H10,H11,H12,H13,H14,H15,"
                "I0,I1,I2,I3,I4,I5,I6,I7,I9,I10");

}

uint8_t* initSdRam () {
    // set up 16 MB SDRAM
    enum {CR1=0x140,CR2=0x144,TR1=0x148,TR2=0x14C,CMR=0x150,RTR=0x154,SR=0x158};
    FMC[CR1] = (0<<13)|(1<<12)|(2<<10)|(3<<7)|(1<<6)|(2<<4)|(1<<2)|(0<<0);
    FMC[TR1] = (1<<24)|(1<<20)|(1<<16)|(6<<12)|(3<<8)|(6<<4)|(1<<0);

    // SDRAM commands
    auto fmcWait = []() { while (FMC[SR](5)) {} };
    fmcWait(); FMC[CMR] = (1<<4)|(1<<0); sys::wait(10); // clock enable
    fmcWait(); FMC[CMR] = (1<<4)|(2<<0);                // precharge
    fmcWait(); FMC[CMR] = (1<<4)|(3<<0);                // auto-refresh
    fmcWait(); FMC[CMR] = (0x231<<9)|(1<<4)|(4<<0);     // load mode
    fmcWait(); FMC[RTR] = (51<<1);                      // refresh rate

    return (uint8_t*) 0xC000'0000;
}

void initBoard () {
    cycles::init();
    fastClock();
    led1.mode("P"); // green LED
    led2.mode("P"); // red LED

    console.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                       UART_FREQ, Irq::UART_NAME, UART_CONF });
    rtc::init();
    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz, %d cy (20%02d/%02d/%02d %02d:%02d:%02d.%03d)",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000, cycles::count(),
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, (dt.ff * 1000) / 256);

    if (rtc::getSecs() < 367 * 86400) {
        logf("rtc set to: %s %s", __DATE__, __TIME__);
        rtc::set(DateTime {});
    }

    initFmcPins();
}

extern "C" int _write (int, char* ptr, int len) {
    Message m { console.dId, 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}
