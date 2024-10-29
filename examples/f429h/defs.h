// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "ram"

//CG[ board leds
#define LED  "B0"
#define LED1 "B0"
#define LED2 "B1"
#define LED3 "C13"
#define LED4 "A4"
//CG]

constexpr Pin ledL (LED1), ledR (LED2), ledC (LED3), ledB (LED4);

//CG[ board uart
#define UART_NAME  USART1
#define UART_PINS  "A9:7,A10"
#define UART_FREQ  90
#define UART_TYPE  USART1.ADDR, DMA2.ADDR, 7-0, 5-0
#define UART_CONF  { ena::USART1, 90, Irq::USART1, \
                     Irq::DMA2_Stream7, Irq::DMA2_Stream5, { 2-1,4,4 } }
//CG]

uart::Sync<UART_TYPE> console (UART_CONF);

void initBoard () {
    fastClock();
    cycles::init();

    ledL.mode("P"); ledL = 1; // inverted logic
    ledR.mode("P"); ledR = 1; // inverted logic
    ledC.mode("P"); ledC = 1; // inverted logic
    ledB.mode("P");

    console.init(UART_PINS, 912'600);
    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}

extern "C" int _write (int, char* ptr, int len) {
    console.transfer(true, (uint8_t*) ptr, len);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}

void initFmcPins () {
    RCC(ena::FMC, 1) = 1;
    Pin::config("B7:V12,C0,C3,"
                "D0,D1,D4,D5,D6,D7,D8,D9,D10,D11,D12,D13,D14,D15,"
                "E0,E1,E2,E3,E4,E5,E7,E8,E9,E10,E11,E12,E13,E14,E15,"
                "F0,F1,F2,F3,F4,F5,F11,F12,F13,F14,F15,"
                "G0,G1,G2,G3,G4,G5,G8,G9,G10,G12,G15,"
                "H2,H3,H5");
}

uint8_t* initPsRam () {
    // set up 4 MB PSRAM
    enum { BCR4=0x18, BTR4=0x1C };
    FMC[BCR4] = (1<<12) | (1<<7) | (1<<4) | (1<<2); // WREN b7 MWID MTYP
    FMC[BTR4] = (1<<16) | (8<<8) | (6<<0); // BUSTURN DATAST ADDSET
    FMC[BCR4](0) = 1; // MBKEN
    return (uint8_t*) 0x6C00'0000;
}

uint8_t* initSdRam () {
    // set up 32 MB SDRAM
    enum {CR1=0x140,CR2=0x144,TR1=0x148,TR2=0x14C,CMR=0x150,RTR=0x154,SR=0x158};
    FMC[CR1] = (1<<13)|(1<<12)|(2<<10)|(3<<7)|(1<<6)|(1<<4)|(2<<2)|(1<<0);
    FMC[TR1] = (1<<24)|(1<<20)|(1<<16)|(5<<12)|(3<<8)|(6<<4)|(1<<0);

    // SDRAM commands
    auto fmcWait = []() { while (FMC[SR](5)) {} };
    fmcWait(); FMC[CMR] = (1<<4)|(1<<0); cycles::msBusy(10); // clock enable
    fmcWait(); FMC[CMR] = (1<<4)|(2<<0);                // precharge
    fmcWait(); FMC[CMR] = (1<<4)|(3<<0);                // auto-refresh
    fmcWait(); FMC[CMR] = (0x231<<9)|(1<<4)|(4<<0);     // load mode
    fmcWait(); FMC[RTR] = (51<<1);                      // refresh rate

    return (uint8_t*) 0xC000'0000;
}
