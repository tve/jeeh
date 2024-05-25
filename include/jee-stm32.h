// Definitions for STM32 microcontrollers (see also src/stm32.cpp).
// Lines with "CG" control the code-generated parts of this file.

//CG[ svd ioregs
constexpr IoReg<0x4001'2400> ADC;
constexpr IoReg<0x5800'1800> AES;
constexpr IoReg<0x4001'0200> COMP;
constexpr IoReg<0x4002'3000> CRC;
constexpr IoReg<0x4000'7400> DAC;
constexpr IoReg<0xE004'2000> DBGMCU;
constexpr IoReg<0x4002'0000> DMA1;
constexpr IoReg<0x4002'0400> DMA2;
constexpr IoReg<0x4002'0800> DMAMUX;
constexpr IoReg<0x5800'0800> EXTI;
constexpr IoReg<0x5800'4000> FLASH;
constexpr IoReg<0x4800'0000> GPIOA;
constexpr IoReg<0x4800'0400> GPIOB;
constexpr IoReg<0x4800'0800> GPIOC;
constexpr IoReg<0x4800'1C00> GPIOH;
constexpr IoReg<0x5800'1400> HSEM;
constexpr IoReg<0x4000'5400> I2C1;
constexpr IoReg<0x4000'5800> I2C2;
constexpr IoReg<0x4000'5C00> I2C3;
constexpr IoReg<0x4000'3000> IWDG;
constexpr IoReg<0x4000'7C00> LPTIM1;
constexpr IoReg<0x4000'9400> LPTIM2;
constexpr IoReg<0x4000'9800> LPTIM3;
constexpr IoReg<0x4000'8000> LPUART;
constexpr IoReg<0xE000'ED90> MPU;
constexpr IoReg<0xE000'E100> NVIC;
constexpr IoReg<0xE000'EF00> NVIC_STIR;
constexpr IoReg<0x5800'2000> PKA;
constexpr IoReg<0x5800'0400> PWR;
constexpr IoReg<0x5800'0000> RCC;
constexpr IoReg<0x5800'1000> RNG;
constexpr IoReg<0x4000'2800> RTC;
constexpr IoReg<0xE000'ED00> SCB;
constexpr IoReg<0xE000'E008> SCB_ACTRL;
constexpr IoReg<0x4001'3000> SPI1;
constexpr IoReg<0x4000'3800> SPI2;
constexpr IoReg<0x5801'0000> SPI3;
constexpr IoReg<0xE000'E010> STK;
constexpr IoReg<0x4001'0000> SYSCFG;
constexpr IoReg<0x4001'0100> SYSCFG_CONTINUE;
constexpr IoReg<0x4000'B000> TAMP;
constexpr IoReg<0x4001'2C00> TIM1;
constexpr IoReg<0x4000'0000> TIM2;
constexpr IoReg<0x4001'4400> TIM16;
constexpr IoReg<0x4001'4800> TIM17;
constexpr IoReg<0x4001'3800> USART1;
constexpr IoReg<0x4000'4400> USART2;
constexpr IoReg<0x4001'0030> VREFBUF;
constexpr IoReg<0x4000'2C00> WWDG;
//CG]

enum struct Irq : uint8_t {
    //CG[ svd irqs
    ADC                    =  18,
    AES                    =  51,
    COMP                   =  21,
    DAC                    =  19,
    DMA1_CH1               =  11,
    DMA1_CH2               =  12,
    DMA1_CH3               =  13,
    DMA1_CH4               =  14,
    DMA1_CH5               =  15,
    DMA1_CH6               =  16,
    DMA1_CH7               =  17,
    DMA2_CH1               =  54,
    DMA2_CH2               =  55,
    DMA2_CH3               =  56,
    DMA2_CH4               =  57,
    DMA2_CH5               =  58,
    DMA2_CH6               =  59,
    DMA2_CH7               =  60,
    DMAMUX1_OVR            =  61,
    EXTI0                  =   6,
    EXTI1                  =   7,
    EXTI2                  =   8,
    EXTI3                  =   9,
    EXTI4                  =  10,
    EXTI9_5                =  22,
    EXTI15_10              =  41,
    FLASH                  =   4,
    HSEM                   =  47,
    I2C1_ER                =  31,
    I2C1_EV                =  30,
    I2C2_ER                =  33,
    I2C2_EV                =  32,
    I2C3_ER                =  49,
    I2C3_EV                =  48,
    LPTIM1                 =  39,
    LPTIM2                 =  40,
    LPTIM3                 =  43,
    LPUART1                =  38,
    PKA                    =  53,
    PVD_PVM_3              =   1,
    Radio_IRQ_Busy         =  50,
    RCC                    =   5,
    RTC_ALARM              =  42,
    RTC_WKUP               =   3,
    SPI1                   =  34,
    SPI2S2                 =  35,
    TAMP_RTCSTAMP_LSECSS_RTCSSRU =   2,
    TIM1_BRK               =  23,
    TIM1_CC                =  26,
    TIM1_TRG_COM           =  25,
    TIM1_UP                =  24,
    TIM2                   =  27,
    TIM16                  =  28,
    TIM17                  =  29,
    True_RNG               =  52,
    USART1                 =  36,
    USART2                 =  37,
    WWDG                   =   0,
    limit = 62,
    //CG]
};

enum : uint16_t {
    //CG[ svd rccs
    AHB1ENR  = 0x48,
    AHB2ENR  = 0x4C,
    AHB3ENR  = 0x50,
    APB1ENR1 = 0x58,
    APB1ENR2 = 0x5C,
    APB2ENR  = 0x60,
    APB3ENR  = 0x64,
    //CG]
};

namespace ena {
    enum : uint16_t {
        //CG[ svd enables
        ADC           =  9 + 8 * APB2ENR,
        AES           = 17 + 8 * AHB3ENR,
        CRC           = 12 + 8 * AHB1ENR,
        DAC1          = 29 + 8 * APB1ENR1,
        DMA1          =  0 + 8 * AHB1ENR,
        DMA2          =  1 + 8 * AHB1ENR,
        DMAMUX1       =  2 + 8 * AHB1ENR,
        FLASH         = 25 + 8 * AHB3ENR,
        GPIOA         =  0 + 8 * AHB2ENR,
        GPIOB         =  1 + 8 * AHB2ENR,
        GPIOC         =  2 + 8 * AHB2ENR,
        GPIOH         =  7 + 8 * AHB2ENR,
        HSEM          = 19 + 8 * AHB3ENR,
        I2C1          = 21 + 8 * APB1ENR1,
        I2C2          = 22 + 8 * APB1ENR1,
        I2C3          = 23 + 8 * APB1ENR1,
        LPTIM1        = 31 + 8 * APB1ENR1,
        LPTIM2        =  5 + 8 * APB1ENR2,
        LPTIM3        =  6 + 8 * APB1ENR2,
        LPUART1       =  0 + 8 * APB1ENR2,
        PKA           = 16 + 8 * AHB3ENR,
        RNG           = 18 + 8 * AHB3ENR,
        RTCAPB        = 10 + 8 * APB1ENR1,
        SPI1          = 12 + 8 * APB2ENR,
        SPI2S2        = 14 + 8 * APB1ENR1,
        SUBGHZSPI     =  0 + 8 * APB3ENR,
        TIM1          = 11 + 8 * APB2ENR,
        TIM16         = 17 + 8 * APB2ENR,
        TIM17         = 18 + 8 * APB2ENR,
        TIM2          =  0 + 8 * APB1ENR1,
        USART1        = 14 + 8 * APB2ENR,
        USART2        = 17 + 8 * APB1ENR1,
        WWDG          = 11 + 8 * APB1ENR1,
        //CG]
    };
} // namespace ena
