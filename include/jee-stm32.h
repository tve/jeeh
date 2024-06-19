// Definitions for STM32 microcontrollers (see also src/stm32.cpp).
// Lines with "CG" control the code-generated parts of this file.

//CG[ svd ioregs
constexpr IoReg<0x5004'0000> ADC1;
constexpr IoReg<0x5004'0100> ADC2;
constexpr IoReg<0x5004'0200> ADC3;
constexpr IoReg<0x5004'0300> ADC123_COMMON;
constexpr IoReg<0x5006'0000> AES;
constexpr IoReg<0x4000'6400> CAN1;
constexpr IoReg<0x4001'0200> COMP;
constexpr IoReg<0x4002'3000> CRC;
constexpr IoReg<0x4000'6000> CRS;
constexpr IoReg<0x4000'7400> DAC1;
constexpr IoReg<0xE004'2000> DBGMCU;
constexpr IoReg<0x4001'6000> DFSDM;
constexpr IoReg<0x4002'0000> DMA1;
constexpr IoReg<0x4002'0400> DMA2;
constexpr IoReg<0x4001'0400> EXTI;
constexpr IoReg<0x4001'1C00> FIREWALL;
constexpr IoReg<0x4002'2000> FLASH;
constexpr IoReg<0xE000'EF34> FPU;
constexpr IoReg<0xE000'ED88> FPU_CPACR;
constexpr IoReg<0x4800'0000> GPIOA;
constexpr IoReg<0x4800'0400> GPIOB;
constexpr IoReg<0x4800'0800> GPIOC;
constexpr IoReg<0x4800'0C00> GPIOD;
constexpr IoReg<0x4800'1000> GPIOE;
constexpr IoReg<0x4800'1C00> GPIOH;
constexpr IoReg<0x4000'5400> I2C1;
constexpr IoReg<0x4000'5800> I2C2;
constexpr IoReg<0x4000'5C00> I2C3;
constexpr IoReg<0x4000'8400> I2C4;
constexpr IoReg<0x4000'3000> IWDG;
constexpr IoReg<0x4000'2400> LCD;
constexpr IoReg<0x4000'7C00> LPTIM1;
constexpr IoReg<0x4000'9400> LPTIM2;
constexpr IoReg<0x4000'8000> LPUART1;
constexpr IoReg<0xE000'ED90> MPU;
constexpr IoReg<0xE000'E100> NVIC;
constexpr IoReg<0xE000'EF00> NVIC_STIR;
constexpr IoReg<0x4000'7800> OPAMP;
constexpr IoReg<0x4000'7000> PWR;
constexpr IoReg<0xA000'1000> QUADSPI;
constexpr IoReg<0x4002'1000> RCC;
constexpr IoReg<0x5006'0800> RNG;
constexpr IoReg<0x4000'2800> RTC;
constexpr IoReg<0x4001'5400> SAI1;
constexpr IoReg<0xE000'ED00> SCB;
constexpr IoReg<0xE000'E008> SCB_ACTRL;
constexpr IoReg<0x4001'2800> SDMMC;
constexpr IoReg<0x4001'3000> SPI1;
constexpr IoReg<0x4000'3800> SPI2;
constexpr IoReg<0x4000'3C00> SPI3;
constexpr IoReg<0xE000'E010> STK;
constexpr IoReg<0x4000'8800> SWPMI1;
constexpr IoReg<0x4001'0000> SYSCFG;
constexpr IoReg<0x4001'2C00> TIM1;
constexpr IoReg<0x4000'0000> TIM2;
constexpr IoReg<0x4000'0400> TIM3;
constexpr IoReg<0x4000'1000> TIM6;
constexpr IoReg<0x4000'1400> TIM7;
constexpr IoReg<0x4001'4000> TIM15;
constexpr IoReg<0x4001'4400> TIM16;
constexpr IoReg<0x4002'4000> TSC;
constexpr IoReg<0x4000'4C00> UART4;
constexpr IoReg<0x4001'3800> USART1;
constexpr IoReg<0x4000'4400> USART2;
constexpr IoReg<0x4000'4800> USART3;
constexpr IoReg<0x4000'6800> USB_FS;
constexpr IoReg<0x4000'6C00> USB_SRAM;
constexpr IoReg<0x4001'0030> VREFBUF;
constexpr IoReg<0x4000'2C00> WWDG;
//CG]

enum struct Irq : uint8_t {
    //CG[ svd irqs
    ADC1                   =  18,
    ADC3                   =  47,
    AES                    =  79,
    CAN1_RX0               =  20,
    CAN1_RX1               =  21,
    CAN1_SCE               =  22,
    CAN1_TX                =  19,
    COMP                   =  64,
    CRS                    =  82,
    DFSDM1_FLT0            =  61,
    DFSDM1_FLT1            =  62,
    DFSDM1_FLT2            =  63,
    DFSDM1_FLT3            =  42,
    DMA1_CH1               =  11,
    DMA1_CH2               =  12,
    DMA1_CH3               =  13,
    DMA1_CH4               =  14,
    DMA1_CH5               =  15,
    DMA1_CH6               =  16,
    DMA1_CH7               =  17,
    DMA2_CH1               =  56,
    DMA2_CH2               =  57,
    DMA2_CH3               =  58,
    DMA2_CH4               =  59,
    DMA2_CH5               =  60,
    DMA2_CH6               =  68,
    DMA2_CH7               =  69,
    EXTI0                  =   6,
    EXTI1                  =   7,
    EXTI2                  =   8,
    EXTI3                  =   9,
    EXTI4                  =  10,
    EXTI9_5                =  23,
    EXTI15_10              =  40,
    FLASH                  =   4,
    FPU                    =  81,
    I2C1_ER                =  32,
    I2C1_EV                =  31,
    I2C2_ER                =  34,
    I2C2_EV                =  33,
    I2C3_ER                =  73,
    I2C3_EV                =  72,
    I2C4_ER                =  84,
    I2C4_EV                =  83,
    LCD                    =  78,
    LPTIM1                 =  65,
    LPTIM2                 =  66,
    LPUART1                =  70,
    PVD_PVM                =   1,
    QUADSPI                =  71,
    RCC                    =   5,
    RNG                    =  80,
    RTC_ALARM              =  41,
    RTC_TAMP_STAMP         =   2,
    RTC_WKUP               =   3,
    SAI1                   =  74,
    SDMMC1                 =  49,
    SPI1                   =  35,
    SPI2                   =  36,
    SPI3                   =  51,
    SWPMI1                 =  76,
    TIM1_BRK_TIM15         =  24,
    TIM1_CC                =  27,
    TIM1_TRG_COM           =  26,
    TIM1_UP_TIM16          =  25,
    TIM2                   =  28,
    TIM3                   =  29,
    TIM6_DACUNDER          =  54,
    TIM7                   =  55,
    TSC                    =  77,
    UART4                  =  52,
    USART1                 =  37,
    USART2                 =  38,
    USART3                 =  39,
    USB_FS                 =  67,
    WWDG                   =   0,
    limit = 85,
    //CG]
#if STM32L0
    DMA1_Channel2 = DMA1_Channel2_3,
    DMA1_Channel3 = DMA1_Channel2_3,
    DMA1_Channel4 = DMA1_Channel4_7,
    DMA1_Channel5 = DMA1_Channel4_7,
    DMA1_Channel6 = DMA1_Channel4_7,
    DMA1_Channel7 = DMA1_Channel4_7,
#endif
};

enum : uint16_t {
    //CG[ svd rccs
    AHB1ENR  = 0x48,
    AHB2ENR  = 0x4C,
    AHB3ENR  = 0x50,
    APB1ENR1 = 0x58,
    APB1ENR2 = 0x5C,
    APB2ENR  = 0x60,
    //CG]
};

namespace ena {
    enum : uint16_t {
        //CG[ svd enables
        ADC           = 13 + 8 * AHB2ENR,
        AES           = 16 + 8 * AHB2ENR,
        CAN1          = 25 + 8 * APB1ENR1,
        CRC           = 12 + 8 * AHB1ENR,
        CRS           = 24 + 8 * APB1ENR1,
        DAC1          = 29 + 8 * APB1ENR1,
        DFSDM         = 24 + 8 * APB1ENR2,
        DMA1          =  0 + 8 * AHB1ENR,
        DMA2          =  1 + 8 * AHB1ENR,
        FIREWALL      =  7 + 8 * APB2ENR,
        FLASH         =  8 + 8 * AHB1ENR,
        GPIOA         =  0 + 8 * AHB2ENR,
        GPIOB         =  1 + 8 * AHB2ENR,
        GPIOC         =  2 + 8 * AHB2ENR,
        GPIOD         =  3 + 8 * AHB2ENR,
        GPIOE         =  4 + 8 * AHB2ENR,
        GPIOH         =  7 + 8 * AHB2ENR,
        I2C1          = 21 + 8 * APB1ENR1,
        I2C2          = 22 + 8 * APB1ENR1,
        I2C3          = 23 + 8 * APB1ENR1,
        I2C4          =  1 + 8 * APB1ENR2,
        LCD           =  9 + 8 * APB1ENR1,
        LPTIM1        = 31 + 8 * APB1ENR1,
        LPTIM2        =  5 + 8 * APB1ENR2,
        LPUART1       =  0 + 8 * APB1ENR2,
        OPAMP         = 30 + 8 * APB1ENR1,
        PWR           = 28 + 8 * APB1ENR1,
        QSPI          =  8 + 8 * AHB3ENR,
        RNG           = 18 + 8 * AHB2ENR,
        RTCAPB        = 10 + 8 * APB1ENR1,
        SAI1          = 21 + 8 * APB2ENR,
        SDMMC         = 10 + 8 * APB2ENR,
        SPI1          = 12 + 8 * APB2ENR,
        SPI3          = 15 + 8 * APB1ENR1,
        SWPMI1        =  2 + 8 * APB1ENR2,
        SYSCFG        =  0 + 8 * APB2ENR,
        TIM1          = 11 + 8 * APB2ENR,
        TIM15         = 16 + 8 * APB2ENR,
        TIM16         = 17 + 8 * APB2ENR,
        TIM2          =  0 + 8 * APB1ENR1,
        TIM3          =  1 + 8 * APB1ENR1,
        TIM6          =  4 + 8 * APB1ENR1,
        TIM7          =  5 + 8 * APB1ENR1,
        TSC           = 16 + 8 * AHB1ENR,
        UART4         = 19 + 8 * APB1ENR1,
        USART1        = 14 + 8 * APB2ENR,
        USART2        = 17 + 8 * APB1ENR1,
        WWDG          = 11 + 8 * APB1ENR1,
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
