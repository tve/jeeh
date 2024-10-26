// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "tick"

//CG1 board leds
#define LED  "B8"

const Pin led (LED,"P");

//CG[ board uart
#define UART_NAME  USART1
#define UART_PINS  "A9:U7,A10"
#define UART_FREQ  170
#define UART_TYPE  USART1.ADDR, DMA1.ADDR, 1-1, 2-1
#define UART_CONF  { ena::USART1, 170, Irq::USART1, \
                     Irq::DMA1_CH1, Irq::DMA1_CH2, 1-1, 25,24 }
//CG]

//CG[ board spi
#define SPI_NAME  SPI1
#define SPI_PINS  "B5:H5,B4,B3,A11:HP"
#define SPI_FREQ  170
#define SPI_TYPE  SPI1.ADDR,DMA1.ADDR,3-1,4-1
#define SPI_CONF  {ena::SPI1,170,Irq::DMA1_CH3,Irq::DMA1_CH4}, {1-1,11,10}
//CG]

//CG[ board i2c
#define I2C_NAME  I2C1
#define I2C_PINS  "B7:OH4,A15"
#define I2C_FREQ  170
#define I2C_TYPE  I2C1.ADDR,DMA1.ADDR,3-1,4-1
#define I2C_CONF  {ena::I2C1,170,Irq::I2C1_EV,Irq::I2C1_ER}, {1-1,17,16}
//CG]

uint32_t i2cTiming (uint16_t khz, uint16_t mhz =SystemCoreClock/1'000'000) {
    switch (mhz) {
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
      case 160: // MHz
        switch (khz) {
          //CG[ i2c timing 160
          // 160 Mhz: (remove this line to re-generate)
          case  100: return 0x30E0D4C1; // prs 3 tcd 14 tdd 0 scll 193 sclh 212
          case  400: return 0x50901B22; // prs 5 tcd 9 tdd 0 scll 34 sclh 27
          case 1000: return 0x40B00B0E; // prs 4 tcd 11 tdd 0 scll 14 sclh 11
          //CG]
        }
        break;
      case 170: // MHz
        switch (khz) {
          //CG[ i2c timing 170
          // 170 Mhz: (remove this line to re-generate)
          case  100: return 0x3010D4C1; // prs 3 tcd 1 tdd 0 scll 193 sclh 212
          case  400: return 0x0050AACE; // prs 0 tcd 5 tdd 0 scll 206 sclh 170
          case 1000: return 0x00503C49; // prs 0 tcd 5 tdd 0 scll 73 sclh 60
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
        USART2[BRR] = SystemCoreClock / 10'000'000; // 160 MHz CPU clock
        USART2[CR1] = (1<<29) | (1<<3) | (1<<0); // FIFOEN TE UE
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!USART2[ISR](7)) {} // TXE
            USART2[TDR] = ((uint8_t const*) ptr)[i];
        }
        //while (!USART2[serio::ISR](6)) {} // TC
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
    fastClock(); // 160 MHz
    cycles::init();
    rtc::init(false);

    serio::init();

    if (rtc::getSecs() == 0)
        rtc::set(DateTime{}); // set to compile date if RTC was not running

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, (dt.ff * 1000) / 256);
    logf("\t i2c: %s spi: %s uart: %s", I2C_PINS, SPI_PINS, UART_PINS);
}
