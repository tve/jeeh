// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "dcf77"

//CG3 board leds
#define LED  "A5"
#define LED1 "A5"
#define LED2 "A1"

const Pin led (LED,"P");
const Pin led2 (LED2,"P");

//CG[ board uart
#define UART_NAME  USART1
#define UART_PINS  "A9:7,A10"
#define UART_FREQ  72
#define UART_CONF  Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,1-1,2-1,25,24
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
