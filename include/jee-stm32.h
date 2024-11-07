// Definitions for STM32 microcontrollers (see also src/stm32.cpp).
// Lines with "CG" control the code-generated parts of this file.

//CG[ svd ioregs
constexpr IoReg<0x5000'0000> ADC1;
constexpr IoReg<0x5000'0300> ADC1_2;
constexpr IoReg<0x5000'0100> ADC2;
constexpr IoReg<0x4000'6400> CAN;
constexpr IoReg<0x4001'001C> COMP;
constexpr IoReg<0x4002'3000> CRC;
constexpr IoReg<0x4000'7400> DAC1;
constexpr IoReg<0x4000'9800> DAC2;
constexpr IoReg<0xE004'2000> DBGMCU;
constexpr IoReg<0x4002'0000> DMA1;
constexpr IoReg<0x4001'0400> EXTI;
constexpr IoReg<0x4002'2000> FLASH;
constexpr IoReg<0xE000'ED88> FPU;
constexpr IoReg<0x4800'0000> GPIOA;
constexpr IoReg<0x4800'0400> GPIOB;
constexpr IoReg<0x4800'0800> GPIOC;
constexpr IoReg<0x4800'0C00> GPIOD;
constexpr IoReg<0x4800'1400> GPIOF;
constexpr IoReg<0x4000'5400> I2C1;
constexpr IoReg<0x4000'3000> IWDG;
constexpr IoReg<0xE000'E100> NVIC;
constexpr IoReg<0x4001'0038> OPAMP;
constexpr IoReg<0x4000'7000> PWR;
constexpr IoReg<0x4002'1000> RCC;
constexpr IoReg<0x4000'2800> RTC;
constexpr IoReg<0xE000'ED00> SCB;
constexpr IoReg<0x4001'3000> SPI1;
constexpr IoReg<0xE000'E010> STK;
constexpr IoReg<0x4001'0000> SYSCFG;
constexpr IoReg<0x4001'2C00> TIM1;
constexpr IoReg<0x4000'0000> TIM2;
constexpr IoReg<0x4000'0400> TIM3;
constexpr IoReg<0x4000'1000> TIM6;
constexpr IoReg<0x4000'1400> TIM7;
constexpr IoReg<0x4001'4000> TIM15;
constexpr IoReg<0x4001'4400> TIM16;
constexpr IoReg<0x4001'4800> TIM17;
constexpr IoReg<0x4002'4000> TSC;
constexpr IoReg<0x4001'3800> USART1;
constexpr IoReg<0x4000'4400> USART2;
constexpr IoReg<0x4000'4800> USART3;
constexpr IoReg<0x4000'2C00> WWDG;
//CG]

enum struct Irq : uint8_t {
    //CG[ svd irqs
    ADC1_2                 =  18,
    CAN_RX1                =  21,
    CAN_SCE                =  22,
    COMP7                  =  66,
    COMP123                =  64,
    COMP456                =  65,
    DMA1_CH1               =  11,
    DMA1_CH2               =  12,
    DMA1_CH3               =  13,
    DMA1_CH4               =  14,
    DMA1_CH5               =  15,
    DMA1_CH6               =  16,
    DMA1_CH7               =  17,
    EXTI0                  =   6,
    EXTI1                  =   7,
    EXTI2_TSC              =   8,
    EXTI3                  =   9,
    EXTI4                  =  10,
    EXTI9_5                =  23,
    EXTI15_10              =  40,
    FLASH                  =   4,
    FPU                    =  81,
    I2C1_ER                =  32,
    I2C1_EV                =  31,
    PVD                    =   1,
    RCC                    =   5,
    RTC_WKUP               =   3,
    RTCAlarm               =  41,
    SPI1                   =  35,
    TAMP_STAMP             =   2,
    TIM1_BRK_TIM15         =  24,
    TIM1_CC                =  27,
    TIM1_TRG_COM_TIM17     =  26,
    TIM1_UP_TIM16          =  25,
    TIM2                   =  28,
    TIM3                   =  29,
    TIM6_DACUNDER          =  54,
    TIM7                   =  55,
    UART4                  =  52,
    UART5                  =  53,
    USART1                 =  37,
    USART2                 =  38,
    USART3                 =  39,
    USB_HP_CAN_TX          =  19,
    USB_LP_CAN_RX0         =  20,
    USB_WKUP               =  76,
    WWDG                   =   0,
    limit = 82,
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
        ADC12         = 28 + 8 * AHBENR,
        ADC34         = 29 + 8 * AHBENR,
        CAN           = 25 + 8 * APB1ENR,
        CRC           =  6 + 8 * AHBENR,
        DAC1          = 29 + 8 * APB1ENR,
        DAC2          = 26 + 8 * APB1ENR,
        DMA1          =  0 + 8 * AHBENR,
        FLITF         =  4 + 8 * AHBENR,
        HRTIM1        = 29 + 8 * APB2ENR,
        I2C1          = 21 + 8 * APB1ENR,
        IOPA          = 17 + 8 * AHBENR,
        IOPB          = 18 + 8 * AHBENR,
        IOPC          = 19 + 8 * AHBENR,
        IOPD          = 20 + 8 * AHBENR,
        IOPF          = 22 + 8 * AHBENR,
        PWR           = 28 + 8 * APB1ENR,
        SPI1          = 12 + 8 * APB2ENR,
        SRAM          =  2 + 8 * AHBENR,
        SYSCFG        =  0 + 8 * APB2ENR,
        TIM1          = 11 + 8 * APB2ENR,
        TIM15         = 16 + 8 * APB2ENR,
        TIM16         = 17 + 8 * APB2ENR,
        TIM17         = 18 + 8 * APB2ENR,
        TIM2          =  0 + 8 * APB1ENR,
        TIM3          =  1 + 8 * APB1ENR,
        TIM6          =  4 + 8 * APB1ENR,
        TIM7          =  5 + 8 * APB1ENR,
        TSC           = 24 + 8 * AHBENR,
        USART1        = 14 + 8 * APB2ENR,
        USART2        = 17 + 8 * APB1ENR,
        USART3        = 18 + 8 * APB1ENR,
        WWDG          = 11 + 8 * APB1ENR,
        //CG]
// missing info, not properly auto-generated from SVD:
#if STM32F3
        GPIOA         = 17 + 8 * AHBENR,
#elif STM32F4 // F413
        UART9         =  6 + 8 * APB2ENR,
        UART10        =  7 + 8 * APB2ENR,
#elif STM32G0
        GPIOA         =  0 + 8 * 0x34,
#elif STM32G4
        DAC1          = 16 + 8 * AHB2ENR,
        DAC2          = 17 + 8 * AHB2ENR,
        DAC3          = 18 + 8 * AHB2ENR,
        DAC4          = 19 + 8 * AHB2ENR,
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
