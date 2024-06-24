// Lines with "CG" control the code-generated parts of this file.

//CG pio

//CG[ board leds
#define LED  "A5"
#define LED1 "A5"
#define LED2 "A7"
#define LED3 "B1"
//CG]

//CG[ board uart
#define UART_NAME  USART6
#define UART_PINS  "C6:8,C7"
#define UART_FREQ  108
#define UART_CONF  Irq::DMA2_Stream6,Irq::DMA2_Stream2,2-1,6-0,2-0,5,5
//CG]

//CG[ board uart_l
#define UART_L_NAME  USART2
#define UART_L_PINS  "A2:7,A3"
#define UART_L_FREQ  108
#define UART_L_CONF  Irq::DMA1_Stream6,Irq::DMA1_Stream5,1-1,6-0,5-0,4,4
//CG]

//CG[ board uart_w
#define UART_W_NAME  UART5
#define UART_W_PINS  "C12:8,D2"
#define UART_W_FREQ  54
#define UART_W_CONF  Irq::DMA1_Stream7,Irq::DMA1_Stream0,1-1,7-0,0-0,4,4
//CG]

//CG[ board spi
#define SPI_NAME  SPI2
#define SPI_PINS  "C3:5,C2,I1,I0:P"
#define SPI_FREQ  54
#define SPI_TYPE  SPI2.ADDR,DMA1.ADDR,4-0,3-0
#define SPI_CONF  { ena::SPI2,54,Irq::DMA1_Stream4,Irq::DMA1_Stream3,1-1,0,0 }
//CG]

//CG2 board rfm69
#define RFM69_DIOS "A4:F,B0,B11,H4,H5"
#define RFM69_NRST "F11"

constexpr Pin led      (LED); // redundant, same as blue
constexpr Pin blueLed  (LED1);
constexpr Pin redLed   (LED2);
constexpr Pin greenLed (LED3);

inline Uart uart ('U');
inline Uart uart_l ('L');
inline Uart uart_w ('W');

constexpr Pin nrst (RFM69_NRST);
Pin dios [5];

extern "C" int _write (int, char* ptr, int len) {
    Message m { uart.dId, 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}

void espPower (bool on) {
    Pin::config(on ? "D3:F" : "D3:P"); // ESP8266 CH_PD, power down
}

void initBoard () {
    fastClock();

    Pin::config(RFM69_DIOS, dios, sizeof dios);
    nrst.mode("P");
    //led.mode("P"); // redundant
    blueLed.mode("P");
    redLed.mode("P");
    greenLed.mode("P");

    rtc::init();

    espPower(false);

    // console uart
    uart.init(UART_PINS, 1'000'000,
                { UART_NAME.ADDR, ena::UART_NAME,
                  UART_FREQ, Irq::UART_NAME, UART_CONF });
    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);

    // loopback uart
    uart_l.init(UART_L_PINS, SystemCoreClock / 32,
                { UART_L_NAME.ADDR, ena::UART_L_NAME,
                  UART_L_FREQ, Irq::UART_L_NAME, UART_L_CONF });

    // wifi/esp uart
    uart_w.init(UART_W_PINS, 1'200,
                { UART_W_NAME.ADDR, ena::UART_W_NAME,
                  UART_W_FREQ, Irq::UART_W_NAME, UART_W_CONF });
}

void initFsmcPins () {
    RCC(ena::FMC, 1) = 1;
    Pin::config("D0:V12,D1,D4,D5,D7,D8,D9,D10,D11,D12,D14,D15,"
                "E0,E1,E7,E8,E9,E10,E11,E12,E13,E14,E15,"
                "F0,F1,F2,F3,F4,F5,F12,F13,F14,F15,"
                "G0,G1,G2,G3,G4,G5,G9");
}

void initPsram () {
    enum { BCR1=0x00, BTR1=0x04 };

    FMC[BCR1] = (1<<20) | (1<<19) | (1<<12) | (1<<8) | (1<<7) | (1<<4);
//  FMC[BTR1] = (1<<20) | (6<<8) | (2<<4) | (9<<0);
    FMC[BTR1] = (4<<8) | (9<<0);
    FMC[BCR1](0) = 1; // MBKEN
}
