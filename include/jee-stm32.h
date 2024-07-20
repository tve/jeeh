// Definitions for STM32 microcontrollers (see also src/stm32.cpp).
// Lines with "CG" control the code-generated parts of this file.

//CG[ svd ioregs
constexpr IoReg<0x4001'2400> ADC1;
constexpr IoReg<0x4001'2800> ADC2;
constexpr IoReg<0x4001'3C00> ADC3;
constexpr IoReg<0x4001'0000> AFIO;
constexpr IoReg<0x4000'6C04> BKP;
constexpr IoReg<0x4000'6400> CAN;
constexpr IoReg<0x4002'3000> CRC;
constexpr IoReg<0x4000'7400> DAC;
constexpr IoReg<0xE004'2000> DBG;
constexpr IoReg<0x4002'0000> DMA1;
constexpr IoReg<0x4002'0400> DMA2;
constexpr IoReg<0x4001'0400> EXTI;
constexpr IoReg<0x4002'2000> FLASH;
constexpr IoReg<0xA000'0000> FSMC;
constexpr IoReg<0x4001'0800> GPIOA;
constexpr IoReg<0x4001'0C00> GPIOB;
constexpr IoReg<0x4001'1000> GPIOC;
constexpr IoReg<0x4001'1400> GPIOD;
constexpr IoReg<0x4001'1800> GPIOE;
constexpr IoReg<0x4001'1C00> GPIOF;
constexpr IoReg<0x4001'2000> GPIOG;
constexpr IoReg<0x4000'5400> I2C1;
constexpr IoReg<0x4000'5800> I2C2;
constexpr IoReg<0x4000'3000> IWDG;
constexpr IoReg<0xE000'E100> NVIC;
constexpr IoReg<0x4000'7000> PWR;
constexpr IoReg<0x4002'1000> RCC;
constexpr IoReg<0x4000'2800> RTC;
constexpr IoReg<0xE000'ED00> SCB;
constexpr IoReg<0x4001'8000> SDIO;
constexpr IoReg<0x4001'3000> SPI1;
constexpr IoReg<0x4000'3800> SPI2;
constexpr IoReg<0x4000'3C00> SPI3;
constexpr IoReg<0xE000'E010> STK;
constexpr IoReg<0x4001'2C00> TIM1;
constexpr IoReg<0x4000'0000> TIM2;
constexpr IoReg<0x4000'0400> TIM3;
constexpr IoReg<0x4000'0800> TIM4;
constexpr IoReg<0x4000'0C00> TIM5;
constexpr IoReg<0x4000'1000> TIM6;
constexpr IoReg<0x4000'1400> TIM7;
constexpr IoReg<0x4001'3400> TIM8;
constexpr IoReg<0x4001'4C00> TIM9;
constexpr IoReg<0x4001'5000> TIM10;
constexpr IoReg<0x4001'5400> TIM11;
constexpr IoReg<0x4000'1800> TIM12;
constexpr IoReg<0x4000'1C00> TIM13;
constexpr IoReg<0x4000'2000> TIM14;
constexpr IoReg<0x4000'4C00> UART4;
constexpr IoReg<0x4000'5000> UART5;
constexpr IoReg<0x4001'3800> USART1;
constexpr IoReg<0x4000'4400> USART2;
constexpr IoReg<0x4000'4800> USART3;
constexpr IoReg<0x4000'5C00> USB;
constexpr IoReg<0x4000'2C00> WWDG;
//CG]

enum struct Irq : uint8_t {
    //CG[ svd irqs
    ADC                    =  18,
    ADC3                   =  47,
    CAN1_RX0               =  20,
    CAN1_RX1               =  21,
    CAN1_SCE               =  22,
    CAN1_TX                =  19,
    DMA1_Channel1          =  11,
    DMA1_Channel2          =  12,
    DMA1_Channel3          =  13,
    DMA1_Channel4          =  14,
    DMA1_Channel5          =  15,
    DMA1_Channel6          =  16,
    DMA1_Channel7          =  17,
    DMA2_Channel1          =  56,
    DMA2_Channel2          =  57,
    DMA2_Channel3          =  58,
    DMA2_Channel4_5        =  59,
    EXTI0                  =   6,
    EXTI1                  =   7,
    EXTI2                  =   8,
    EXTI3                  =   9,
    EXTI4                  =  10,
    EXTI9_5                =  23,
    EXTI15_10              =  40,
    FLASH                  =   4,
    FSMC                   =  48,
    I2C1_ER                =  32,
    I2C1_EV                =  31,
    I2C2_ER                =  34,
    I2C2_EV                =  33,
    PVD                    =   1,
    RCC                    =   5,
    RTC                    =   3,
    RTCAlarm               =  41,
    SDIO                   =  49,
    SPI1                   =  35,
    SPI2                   =  36,
    SPI3                   =  51,
    TAMPER                 =   2,
    TIM1_BRK_TIM9          =  24,
    TIM1_CC                =  27,
    TIM1_TRG_COM_TIM11     =  26,
    TIM1_UP_TIM10          =  25,
    TIM2                   =  28,
    TIM3                   =  29,
    TIM4                   =  30,
    TIM5                   =  50,
    TIM6                   =  54,
    TIM7                   =  55,
    TIM8_BRK_TIM12         =  43,
    TIM8_CC                =  46,
    TIM8_TRG_COM_TIM14     =  45,
    TIM8_UP_TIM13          =  44,
    UART4                  =  52,
    UART5                  =  53,
    USART1                 =  37,
    USART2                 =  38,
    USART3                 =  39,
    USB_FS_WKUP            =  42,
    WWDG                   =   0,
    limit = 60,
    //CG]
#if STM32G4
    LPUART1 = LPUART,
#endif
#if STM32L0
    DMA1_Channel2 = DMA1_Channel2_3,
    DMA1_Channel3 = DMA1_Channel2_3,
    DMA1_Channel4 = DMA1_Channel4_7,
    DMA1_Channel5 = DMA1_Channel4_7,
    DMA1_Channel6 = DMA1_Channel4_7,
    DMA1_Channel7 = DMA1_Channel4_7,
    I2C1_EV = I2C1,
    I2C1_ER = I2C1,
#endif
};

enum : uint16_t {
    //CG3 svd rccs
    AHBENR   = 0x14,
    APB1ENR  = 0x1C,
    APB2ENR  = 0x18,
};

namespace ena {
    enum : uint16_t {
        //CG[ svd enables
        ADC1          =  9 + 8 * APB2ENR,
        ADC2          = 10 + 8 * APB2ENR,
        ADC3          = 15 + 8 * APB2ENR,
        AFIO          =  0 + 8 * APB2ENR,
        BKP           = 27 + 8 * APB1ENR,
        CAN           = 25 + 8 * APB1ENR,
        CRC           =  6 + 8 * AHBENR,
        DAC           = 29 + 8 * APB1ENR,
        DMA1          =  0 + 8 * AHBENR,
        DMA2          =  1 + 8 * AHBENR,
        FLITF         =  4 + 8 * AHBENR,
        FSMC          =  8 + 8 * AHBENR,
        I2C1          = 21 + 8 * APB1ENR,
        I2C2          = 22 + 8 * APB1ENR,
        IOPA          =  2 + 8 * APB2ENR,
        IOPB          =  3 + 8 * APB2ENR,
        IOPC          =  4 + 8 * APB2ENR,
        IOPD          =  5 + 8 * APB2ENR,
        IOPE          =  6 + 8 * APB2ENR,
        IOPF          =  7 + 8 * APB2ENR,
        IOPG          =  8 + 8 * APB2ENR,
        PWR           = 28 + 8 * APB1ENR,
        SDIO          = 10 + 8 * AHBENR,
        SPI1          = 12 + 8 * APB2ENR,
        SPI2          = 14 + 8 * APB1ENR,
        SPI3          = 15 + 8 * APB1ENR,
        SRAM          =  2 + 8 * AHBENR,
        TIM1          = 11 + 8 * APB2ENR,
        TIM10         = 20 + 8 * APB2ENR,
        TIM11         = 21 + 8 * APB2ENR,
        TIM12         =  6 + 8 * APB1ENR,
        TIM13         =  7 + 8 * APB1ENR,
        TIM14         =  8 + 8 * APB1ENR,
        TIM2          =  0 + 8 * APB1ENR,
        TIM3          =  1 + 8 * APB1ENR,
        TIM4          =  2 + 8 * APB1ENR,
        TIM5          =  3 + 8 * APB1ENR,
        TIM6          =  4 + 8 * APB1ENR,
        TIM7          =  5 + 8 * APB1ENR,
        TIM8          = 13 + 8 * APB2ENR,
        TIM9          = 19 + 8 * APB2ENR,
        UART4         = 19 + 8 * APB1ENR,
        UART5         = 20 + 8 * APB1ENR,
        USART1        = 14 + 8 * APB2ENR,
        USART2        = 17 + 8 * APB1ENR,
        USART3        = 18 + 8 * APB1ENR,
        USB           = 23 + 8 * APB1ENR,
        WWDG          = 11 + 8 * APB1ENR,
        //CG]
#if STM32F3
        GPIOA         = 17 + 8 * AHBENR,
#elif STM32G0
        GPIOA         =  0 + 8 * 0x34,
#elif STM32L0
        GPIOA         =  0 + 8 * 0x2C,
#endif
    };
} // namespace ena

uint32_t fastClock (bool high =true);
uint32_t slowClock (bool high =true);
uint32_t clockChange (uint32_t hz);

void swoInit (uint32_t baud, uint32_t hz =SystemCoreClock);
void swoWrite (void const* ptr =nullptr, size_t len =0);
