// Definitions for STM32 microcontrollers (see also src/stm32.cpp).
// Lines with "CG" control the code-generated parts of this file.

//CG[ svd ioregs
constexpr IoReg<0x5000'0000> ADC1;
constexpr IoReg<0x5000'0100> ADC2;
constexpr IoReg<0x5000'0300> ADC12_COMMON;
constexpr IoReg<0x5000'0700> ADC345_COMMON;
constexpr IoReg<0x5006'0000> AES;
constexpr IoReg<0x4001'0200> COMP;
constexpr IoReg<0x4002'0C00> CORDIC;
constexpr IoReg<0x4002'3000> CRC;
constexpr IoReg<0x4000'2000> CRS;
constexpr IoReg<0x5000'0800> DAC1;
constexpr IoReg<0x5000'0C00> DAC2;
constexpr IoReg<0x5000'1000> DAC3;
constexpr IoReg<0x5000'1400> DAC4;
constexpr IoReg<0xE004'2000> DBGMCU;
constexpr IoReg<0x4002'0000> DMA1;
constexpr IoReg<0x4002'0400> DMA2;
constexpr IoReg<0x4002'0800> DMAMUX;
constexpr IoReg<0x4001'0400> EXTI;
constexpr IoReg<0x4000'6400> FDCAN1;
constexpr IoReg<0x4000'A400> FDCAN;
constexpr IoReg<0x4002'2000> FLASH;
constexpr IoReg<0x4002'1400> FMAC;
constexpr IoReg<0xE000'EF34> FPU;
constexpr IoReg<0xE000'ED88> FPU_CPACR;
constexpr IoReg<0x4800'0000> GPIOA;
constexpr IoReg<0x4800'0400> GPIOB;
constexpr IoReg<0x4800'0800> GPIOC;
constexpr IoReg<0x4800'0C00> GPIOD;
constexpr IoReg<0x4800'1000> GPIOE;
constexpr IoReg<0x4800'1400> GPIOF;
constexpr IoReg<0x4800'1800> GPIOG;
constexpr IoReg<0x4000'5400> I2C1;
constexpr IoReg<0x4000'5800> I2C2;
constexpr IoReg<0x4000'7800> I2C3;
constexpr IoReg<0x4000'3000> IWDG;
constexpr IoReg<0x4000'7C00> LPTIMER1;
constexpr IoReg<0x4000'8000> LPUART1;
constexpr IoReg<0xE000'E084> MPU;
constexpr IoReg<0xE000'E100> NVIC;
constexpr IoReg<0xE000'EF00> NVIC_STIR;
constexpr IoReg<0x4001'0300> OPAMP;
constexpr IoReg<0x4000'7000> PWR;
constexpr IoReg<0x4002'1000> RCC;
constexpr IoReg<0x5006'0800> RNG;
constexpr IoReg<0x4000'2800> RTC;
constexpr IoReg<0x4001'5400> SAI;
constexpr IoReg<0xE000'ED00> SCB;
constexpr IoReg<0xE000'E008> SCB_ACTLR;
constexpr IoReg<0x4001'3000> SPI1;
constexpr IoReg<0x4000'3800> SPI2;
constexpr IoReg<0x4000'3C00> SPI3;
constexpr IoReg<0xE000'E010> STK;
constexpr IoReg<0x4001'0000> SYSCFG;
constexpr IoReg<0x4000'2400> TAMP;
constexpr IoReg<0x4001'2C00> TIM1;
constexpr IoReg<0x4000'0000> TIM2;
constexpr IoReg<0x4000'0400> TIM3;
constexpr IoReg<0x4000'0800> TIM4;
constexpr IoReg<0x4000'1000> TIM6;
constexpr IoReg<0x4000'1400> TIM7;
constexpr IoReg<0x4001'3400> TIM8;
constexpr IoReg<0x4001'4000> TIM15;
constexpr IoReg<0x4001'4400> TIM16;
constexpr IoReg<0x4001'4800> TIM17;
constexpr IoReg<0x4000'4C00> UART4;
constexpr IoReg<0x4000'A000> UCPD1;
constexpr IoReg<0x4001'3800> USART1;
constexpr IoReg<0x4000'4400> USART2;
constexpr IoReg<0x4000'4800> USART3;
constexpr IoReg<0x4000'5C00> USB_FS_DEVICE;
constexpr IoReg<0x4001'0030> VREFBUF;
constexpr IoReg<0x4000'2C00> WWDG;
//CG]

enum struct Irq : uint8_t {
    //CG[ svd irqs
    ADC1_2                 =  18,
    AES                    =  85,
    COMP1_2_3              =  64,
    COMP4                  =  65,
    Cordic                 = 100,
    CRS                    =  75,
    DMA1_CH1               =  11,
    DMA1_CH2               =  12,
    DMA1_CH3               =  13,
    DMA1_CH4               =  14,
    DMA1_CH5               =  15,
    DMA1_CH6               =  16,
    DMA2_CH1               =  56,
    DMA2_CH2               =  57,
    DMA2_CH3               =  58,
    DMA2_CH4               =  59,
    DMA2_CH5               =  60,
    DMA2_CH6               =  97,
    DMAMUX_OVR             =  94,
    EXTI0                  =   6,
    EXTI1                  =   7,
    EXTI2                  =   8,
    EXTI3                  =   9,
    EXTI4                  =  10,
    EXTI9_5                =  23,
    EXTI15_10              =  40,
    fdcan1_intr0_it        =  22,
    fdcan1_intr1_it        =  21,
    FLASH                  =   4,
    FMAC                   = 101,
    FPU                    =  81,
    I2C1_ER                =  32,
    I2C1_EV                =  31,
    I2C2_ER                =  34,
    I2C2_EV                =  33,
    I2C3_ER                =  93,
    I2C3_EV                =  92,
    LPTIM1                 =  49,
    LPUART                 =  91,
    PVD_PVM                =   1,
    RCC                    =   5,
    RNG                    =  90,
    RTC_ALARM              =  41,
    RTC_TAMP_CSS_LSE       =   2,
    RTC_WKUP               =   3,
    SAI                    =  76,
    SPI1                   =  35,
    SPI2                   =  36,
    SPI3                   =  51,
    TIM1_BRK_TIM15         =  24,
    TIM1_CC                =  27,
    TIM1_TRG_COM           =  26,
    TIM1_UP_TIM16          =  25,
    TIM2                   =  28,
    TIM3                   =  29,
    TIM4                   =  30,
    TIM6_DACUNDER          =  54,
    TIM7                   =  55,
    TIM8_BRK               =  43,
    TIM8_CC                =  46,
    TIM8_TRG_COM           =  45,
    TIM8_UP                =  44,
    UART4                  =  52,
    UCPD1                  =  63,
    USART1                 =  37,
    USART2                 =  38,
    USART3                 =  39,
    USB_HP                 =  19,
    USB_LP                 =  20,
    USBWakeUP              =  42,
    WWDG                   =   0,
    limit = 102,
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
        ADC12         = 13 + 8 * AHB2ENR,
        ADC345        = 14 + 8 * AHB2ENR,
        CORDIC        =  3 + 8 * AHB1ENR,
        CRC           = 12 + 8 * AHB1ENR,
        CRS           =  8 + 8 * APB1ENR1,
        CRYPT         = 24 + 8 * AHB2ENR,
        DMA1          =  0 + 8 * AHB1ENR,
        DMA2          =  1 + 8 * AHB1ENR,
        DMAMUX        =  2 + 8 * AHB1ENR,
        FDCAN         = 25 + 8 * APB1ENR1,
        FLITF         =  8 + 8 * AHB1ENR,
        FMAC          =  4 + 8 * AHB1ENR,
        FMC           =  0 + 8 * AHB3ENR,
        GPIOA         =  0 + 8 * AHB2ENR,
        GPIOB         =  1 + 8 * AHB2ENR,
        GPIOC         =  2 + 8 * AHB2ENR,
        GPIOD         =  3 + 8 * AHB2ENR,
        GPIOE         =  4 + 8 * AHB2ENR,
        GPIOF         =  5 + 8 * AHB2ENR,
        GPIOG         =  6 + 8 * AHB2ENR,
        HRTIMER       = 26 + 8 * APB2ENR,
        I2C1          = 21 + 8 * APB1ENR1,
        I2C2          = 22 + 8 * APB1ENR1,
        I2C4          =  1 + 8 * APB1ENR2,
        LPTIM1        = 31 + 8 * APB1ENR1,
        LPUART1       =  0 + 8 * APB1ENR2,
        PWR           = 28 + 8 * APB1ENR1,
        QUADSPI1      =  8 + 8 * AHB3ENR,
        RNG           = 26 + 8 * AHB2ENR,
        RTCAPB        = 10 + 8 * APB1ENR1,
        SAI1          = 21 + 8 * APB2ENR,
        SP3           = 15 + 8 * APB1ENR1,
        SPI1          = 12 + 8 * APB2ENR,
        SPI2          = 14 + 8 * APB1ENR1,
        SPI4          = 15 + 8 * APB2ENR,
        SYSCFG        =  0 + 8 * APB2ENR,
        TIM1          = 11 + 8 * APB2ENR,
        TIM15         = 16 + 8 * APB2ENR,
        TIM16         = 17 + 8 * APB2ENR,
        TIM17         = 18 + 8 * APB2ENR,
        TIM2          =  0 + 8 * APB1ENR1,
        TIM20         = 20 + 8 * APB2ENR,
        TIM3          =  1 + 8 * APB1ENR1,
        TIM4          =  2 + 8 * APB1ENR1,
        TIM5          =  3 + 8 * APB1ENR1,
        TIM6          =  4 + 8 * APB1ENR1,
        TIM7          =  5 + 8 * APB1ENR1,
        TIM8          = 13 + 8 * APB2ENR,
        UART4         = 19 + 8 * APB1ENR1,
        UART5         = 20 + 8 * APB1ENR1,
        USART1        = 14 + 8 * APB2ENR,
        USART2        = 17 + 8 * APB1ENR1,
        USART3        = 18 + 8 * APB1ENR1,
        USBD          = 23 + 8 * APB1ENR1,
        USBPD         =  8 + 8 * APB1ENR2,
        WWDG          = 11 + 8 * APB1ENR1,
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
