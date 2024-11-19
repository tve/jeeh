// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "tick"

//CG1 board leds
#define LED  "B3"

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:7,A15:3"
#define UART_FREQ  80
#define UART_TYPE  USART2.ADDR, DMA1.ADDR, 7-1, 6-1
#define UART_CONF  { ena::USART2, 80, Irq::USART2, \
                     Irq::DMA1_Channel7, Irq::DMA1_Channel6, { 1-1,2,2 } }
#define UART_INSTALL(w) extern "C" { \
    void USART2_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Stream7_IRQHandler () { (w).irqDma(); } \
    void DMA1_Stream6_IRQHandler () { (w).irqDma(); } \
}
//CG]

//CG[ board spi
#define SPI_NAME  SPI1
#define SPI_PINS  "B5:H5,B4,B3,A11:HP"
#define SPI_FREQ  80
#define SPI_TYPE  SPI1.ADDR, DMA1.ADDR, 3-1, 2-1
#define SPI_CONF  { ena::SPI1, 80, \
                    Irq::DMA1_Channel3, Irq::DMA1_Channel2, { 1-1,1,1 } }
//CG]

//CG[ board i2c
#define I2C_NAME  I2C1
#define I2C_PINS  "B7:OH4,B6"
#define I2C_FREQ  80
#define I2C_TYPE  I2C1.ADDR, DMA2.ADDR, 7-1, 6-1
#define I2C_CONF  { ena::I2C1, 80, Irq::I2C1_EV, Irq::I2C1_ER, \
                    Irq::DMA2_CH7, Irq::DMA2_CH6, { 2-1,5,5 } }
//CG]

uint32_t i2cTiming (uint16_t khz, uint16_t mhz =SystemCoreClock/1'000'000) {
    switch (mhz) {
      case 4: // MHz
        switch (khz) {
          //CG[ i2c timing 4
          // 4 Mhz: (remove this line to re-generate)
          case  100: return 0x00101110; // prs 0 tcd 1 tdd 0 scll 16 sclh 17
          case  400: return 0x00100003; // prs 0 tcd 1 tdd 0 scll 3 sclh 0
          // no valid solution for 1000 kHz @ 4 MHz
          //CG]
        }
        break;
      case 16: // MHz
        switch (khz) {
          //CG[ i2c timing 16
          // 16 Mhz: (remove this line to re-generate)
          case  100: return 0x00504F49; // prs 0 tcd 5 tdd 0 scll 73 sclh 79
          case  400: return 0x00500D12; // prs 0 tcd 5 tdd 0 scll 18 sclh 13
          case 1000: return 0x00500205; // prs 0 tcd 5 tdd 0 scll 5 sclh 2
          //CG]
        }
        break;
      case 48: // MHz
        switch (khz) {
          //CG[ i2c timing 48
          // 48 Mhz: (remove this line to re-generate)
          case  100: return 0x0010F1DB; // prs 0 tcd 1 tdd 0 scll 219 sclh 241
          case  400: return 0x00102E39; // prs 0 tcd 1 tdd 0 scll 57 sclh 46
          case 1000: return 0x00100F13; // prs 0 tcd 1 tdd 0 scll 19 sclh 15
          //CG]
        }
        break;
      case 80: // MHz
        switch (khz) {
          //CG[ i2c timing 80
          // 80 Mhz: (remove this line to re-generate)
          case  100: return 0x10E0D4C0; // prs 1 tcd 14 tdd 0 scll 192 sclh 212
          case  400: return 0xA0200609; // prs 10 tcd 2 tdd 0 scll 9 sclh 6
          case 1000: return 0x10E00E11; // prs 1 tcd 14 tdd 0 scll 17 sclh 14
          //CG]
        }
        break;
    }
    fail();
}

namespace serio {
    enum { CR1=0x00, BRR=0x0C, ISR=0x1C, TDR=0x28 };

    void init () {
        Pin::config("A2:7");
        RCC(ena::USART2,1) = 1;
        USART2[BRR] = SystemCoreClock / 2'000'000;
        USART2[CR1] = (1<<3) | (1<<0); // TE UE
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!USART2[ISR](7)) {} // TXE
            USART2[TDR] = ((uint8_t const*) ptr)[i];
        }
        //while (!USART2[serio::ISR](6)) {} // TC
    }
}

Pin led (LED,"P");

void initBoard () {
    cycles::init();
    rtc::init();
#if 0
    RCC[0x00](2) = 1; // MSIPLLEN
    fastClock(false); // 48 MHz MSI
    cycles::msBusy(3); // let MSI PLL stabilise
#else
    fastClock(); // 80 MHz
#endif

    serio::init();

    if (rtc::getSecs() == 0)
        rtc::set(DateTime{}); // set to compile date if RTC was not running

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, dt.ms);
    logf("\t i2c: %s spi: %s uart: %s", I2C_PINS, SPI_PINS, UART_PINS);
}

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1)
        serio::write(buf, len);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    serio::write(ptr, len);
}
