// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "dcf77"

//CG3 board leds
#define LED  "A5"
#define LED1 "A5"
#define LED2 "A1"

//CG[ board dcf
#define DCF_DAT "A4"
#define DCF_PON "B7"
#define DCF_VCC "A6"
#define DCF_GND "A7"
//CG]

//CG[ board msf
#define MSF_DAT "A14"
#define MSF_PON "B7"
#define MSF_VCC "A6"
#define MSF_GND "A7"
//CG]

//CG1 board lipo
#define LIPO_ADC "B0"

//CG2 board can
#define CAN_RX "B8"
#define CAN_TX "B9"

//CG[ board usb
#define USB_N "A11"
#define USB_P "A12"
#define USB_DET "C11"
#define USB_DISC "C12"
//CG]

//CG1 board rtc
#define RTC_REFIN "A1"

//CG[ board uart2 tty
#define UART_TTY_NAME  USART2
#define UART_TTY_PINS  "A2:7,A3"
#define UART_TTY_FREQ  36
#define UART_TTY_CONF  Irq::DMA1_CH7,Irq::DMA1_CH6,1-1,7-1,6-1,7,6
//CG]

//CG[ board uart3 gps
#define UART_GPS_NAME  USART3
#define UART_GPS_PINS  "B10:7,B11"
#define UART_GPS_FREQ  36
#define UART_GPS_CONF  Irq::DMA2_CH2,Irq::DMA2_CH1,2-1,2-1,1-1,2,1
//CG]

//CG2 board gps
#define GPS_PPS "B5"
#define GPS_PWR "F4"

//CG[ board spi1 rfm
#define SPI_RFM_NAME  SPI1
#define SPI_RFM_PINS  "A7:5,A6,A5,A4:P"
#define SPI_RFM_FREQ  72
#define SPI_RFM_TYPE  SPI1.ADDR,DMA1.ADDR,3-1,2-1
#define SPI_RFM_CONF  {ena::SPI1,72,Irq::DMA1_CH3,Irq::DMA1_CH2}, {1-1,3,2}
//CG]

//CG3 board rfm
#define RFM_D0 "C0"
#define RFM_D3 "C1"
#define RFM_RST "C2"

//CG[ board spi2 sd
#define SPI_SD_NAME  SPI2
#define SPI_SD_PINS  "B15:5,B14,B13,D2:P"
#define SPI_SD_FREQ  36
#define SPI_SD_TYPE  SPI2.ADDR,DMA1.ADDR,5-1,4-1
#define SPI_SD_CONF  {ena::SPI2,36,Irq::DMA1_CH5,Irq::DMA1_CH4}, {1-1,5,4}
//CG]

uint32_t i2cTiming (uint16_t khz, uint16_t mhz =SystemCoreClock/1'000'000) {
    switch (mhz) {
      case 8: // MHz
        switch (khz) {
          //CG[ i2c timing 8
          // 8 Mhz: (remove this line to re-generate)
          case  100: return 0x00202523; // prs 0 tcd 2 tdd 0 scll 35 sclh 37
          case  400: return 0x00200408; // prs 0 tcd 2 tdd 0 scll 8 sclh 4
          // no valid solution for 1000 kHz @ 8 MHz
          //CG]
        }
        break;
      case 72: // MHz
        switch (khz) {
          //CG[ i2c timing 72
          // 72 Mhz: (remove this line to re-generate)
          case  100: return 0x40004742; // prs 4 tcd 0 tdd 0 scll 66 sclh 71
          case  400: return 0x00204757; // prs 0 tcd 2 tdd 0 scll 87 sclh 71
          case 1000: return 0x0020181E; // prs 0 tcd 2 tdd 0 scll 30 sclh 24
          //CG]
        }
        break;
    }
    fail();
}

// end of codegen section, the rest is common utility code:

const Pin led (LED,"P");
const Pin led2 (LED2,"P");

Pin dcfData (DCF_DAT,"U");
Pin dcfPon (DCF_PON,"P");

//Ticker ticker;
//TICKER_INSTALL(ticker)

namespace serio {
    enum { CR1=0x00, BRR=0x0C, ISR=0x1C, TDR=0x28 };

    void init () {
        Pin::config(UART_TTY_PINS);
        RCC(ena::UART_TTY_NAME,1) = 1;
        auto mhz = SystemCoreClock / 1'000'000;
        while (mhz > UART_TTY_FREQ)
            mhz /= 2;
        UART_TTY_NAME[BRR] = mhz / 2; // 2 Mbd
        UART_TTY_NAME[CR1] = (1<<3) | (1<<0); // TE UE
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!UART_TTY_NAME[ISR](7)) {} // TXE
            UART_TTY_NAME[TDR] = ((uint8_t const*) ptr)[i];
        }
        //while (!UART_TTY_NAME[serio::ISR](6)) {} // TC
    }
}

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1)
        serio::write(buf, len);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    serio::write(ptr, len);
}

void initBoard () {
    fastClock(); // 72 MHz
    serio::init();
    cycles::init();
    rtc::init();
    //ticker.init();

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, (dt.ff * 1000) / 256);

#ifdef DCF_VCC
    Pin vcc (DCF_VCC,"P"), gnd (DCF_GND,"P");
    vcc = 1;
#endif
}
