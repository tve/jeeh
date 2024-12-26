// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "g431k"

//CG1 board leds
#define LED  "B8"

//CG3 board pins
#define PINS_ADC "A1"
#define PINS_ACH (2)
#define PINS_DAC "A4"

const Pin led (LED,"P");

//CG[ board uart
#define UART_NAME USART1
#define UART_TRIGGER(w) extern "C" { \
    void DMA1_Channel1_IRQHandler () { (w).irqTx(); } \
    void DMA1_Channel2_IRQHandler () { (w).irqRx(); } \
    void USART1_IRQHandler () { (w).irqRx(); } \
}
constexpr uart::Config UART_CONF {
    "A9:U7,A10", USART1.ADDR, ena::USART1, 170,
    DMA1.ADDR, 1-1, 1-1,2-1, 25,24,
    Irq::DMA1_CH1, Irq::DMA1_CH2, Irq::USART1, 64,
};
//CG]

//CG[ board spi
#define SPI_NAME SPI1
#define SPI_TRIGGER(w) extern "C" { \
    void DMA1_Channel3_IRQHandler () { (w).irqDma(); } \
    void DMA1_Channel4_IRQHandler () { (w).irqDma(); } \
}
constexpr spi::Config SPI_CONF {
    "B5:V5,B4,B3,A11:HP", SPI1.ADDR, ena::SPI1, 170,
    DMA1.ADDR, 1-1, 3-1,4-1, 11,10,
    Irq::DMA1_CH3, Irq::DMA1_CH4,
};
//CG]

//CG[ board i2c
#define I2C_NAME I2C1
#define I2C_TRIGGER(w) extern "C" { \
    void I2C1_EV_IRQHandler () { (w).irqI2c(); } \
}
constexpr i2c::Config I2C_CONF {
    "B7:OH4,A15", I2C1.ADDR, ena::I2C1, 170,
    DMA1.ADDR, 1-1, 5-1,6-1, 17,16,
    Irq::DMA1_CH5, Irq::DMA1_CH6, Irq::I2C1_EV, Irq::I2C1_ER,
};
//CG]
