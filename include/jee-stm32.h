// Definitions for STM32 microcontrollers (see also src/stm32.cpp).
// Lines with "CG" control the code-generated parts of this file.

//CG[ svd ioregs
constexpr IoReg<0xE000'EF90> AC;
constexpr IoReg<0x4001'2000> ADC1;
constexpr IoReg<0x4001'2100> ADC2;
constexpr IoReg<0x4001'2200> ADC3;
constexpr IoReg<0x4001'2300> C_ADC;
constexpr IoReg<0x4000'6400> CAN1;
constexpr IoReg<0x4002'3000> CRC;
constexpr IoReg<0x5006'0000> CRYP;
constexpr IoReg<0x4000'7400> DAC;
constexpr IoReg<0xE004'2000> DBG;
constexpr IoReg<0x4002'6000> DMA1;
constexpr IoReg<0x4002'6400> DMA2;
constexpr IoReg<0x4001'3C00> EXTI;
constexpr IoReg<0x4002'3C00> FLASH;
constexpr IoReg<0xA000'0000> FMC;
constexpr IoReg<0xE000'EF34> FPU;
constexpr IoReg<0xE000'ED88> FPU_CPACR;
constexpr IoReg<0x4002'0000> GPIOA;
constexpr IoReg<0x4002'0400> GPIOB;
constexpr IoReg<0x4002'0800> GPIOC;
constexpr IoReg<0x4002'0C00> GPIOD;
constexpr IoReg<0x4002'1000> GPIOE;
constexpr IoReg<0x4002'1400> GPIOF;
constexpr IoReg<0x4002'1800> GPIOG;
constexpr IoReg<0x4002'1C00> GPIOH;
constexpr IoReg<0x4002'2000> GPIOI;
constexpr IoReg<0x4000'5400> I2C1;
constexpr IoReg<0x4000'5800> I2C2;
constexpr IoReg<0x4000'5C00> I2C3;
constexpr IoReg<0x4000'3000> IWDG;
constexpr IoReg<0x4000'2400> LPTIM1;
constexpr IoReg<0xE000'ED90> MPU;
constexpr IoReg<0xE000'E100> NVIC;
constexpr IoReg<0xE000'EF00> NVIC_STIR;
constexpr IoReg<0x5000'0800> OTG_FS_DEVICE;
constexpr IoReg<0x5000'0000> OTG_FS_GLOBAL;
constexpr IoReg<0x5000'0400> OTG_FS_HOST;
constexpr IoReg<0x5000'0E00> OTG_FS_PWRCLK;
constexpr IoReg<0x4004'0800> OTG_HS_DEVICE;
constexpr IoReg<0x4004'0000> OTG_HS_GLOBAL;
constexpr IoReg<0x4004'0400> OTG_HS_HOST;
constexpr IoReg<0x4004'0E00> OTG_HS_PWRCLK;
constexpr IoReg<0xE000'ED78> PF;
constexpr IoReg<0x4000'7000> PWR;
constexpr IoReg<0xA000'1000> QUADSPI;
constexpr IoReg<0x4002'3800> RCC;
constexpr IoReg<0x5006'0800> RNG;
constexpr IoReg<0x4000'2800> RTC;
constexpr IoReg<0x4001'5800> SAI1;
constexpr IoReg<0x4001'5C00> SAI2;
constexpr IoReg<0xE000'ED00> SCB;
constexpr IoReg<0xE000'E008> SCB_ACTRL;
constexpr IoReg<0x4001'2C00> SDMMC1;
constexpr IoReg<0x4001'1C00> SDMMC2;
constexpr IoReg<0x4001'3000> SPI1;
constexpr IoReg<0x4000'3800> SPI2;
constexpr IoReg<0x4000'3C00> SPI3;
constexpr IoReg<0x4001'3400> SPI4;
constexpr IoReg<0x4001'5000> SPI5;
constexpr IoReg<0xE000'E010> STK;
constexpr IoReg<0x4001'3800> SYSCFG;
constexpr IoReg<0x4001'0000> TIM1;
constexpr IoReg<0x4000'0000> TIM2;
constexpr IoReg<0x4000'0400> TIM3;
constexpr IoReg<0x4000'0800> TIM4;
constexpr IoReg<0x4000'0C00> TIM5;
constexpr IoReg<0x4000'1000> TIM6;
constexpr IoReg<0x4000'1400> TIM7;
constexpr IoReg<0x4001'0400> TIM8;
constexpr IoReg<0x4001'4000> TIM9;
constexpr IoReg<0x4001'4400> TIM10;
constexpr IoReg<0x4001'4800> TIM11;
constexpr IoReg<0x4000'1800> TIM12;
constexpr IoReg<0x4000'1C00> TIM13;
constexpr IoReg<0x4000'2000> TIM14;
constexpr IoReg<0x4000'4C00> UART4;
constexpr IoReg<0x4000'5000> UART5;
constexpr IoReg<0x4000'7800> UART7;
constexpr IoReg<0x4000'7C00> UART8;
constexpr IoReg<0x4001'1000> USART1;
constexpr IoReg<0x4000'4400> USART2;
constexpr IoReg<0x4000'4800> USART3;
constexpr IoReg<0x4001'1400> USART6;
constexpr IoReg<0x4000'2C00> WWDG;
//CG]

enum struct Irq : uint8_t {
    //CG[ svd irqs
    ADC                    =  18,
    AES                    =  79,
    CAN1_RX0               =  20,
    CAN1_RX1               =  21,
    CAN1_SCE               =  22,
    CAN1_TX                =  19,
    DMA1_Stream0           =  11,
    DMA1_Stream1           =  12,
    DMA1_Stream2           =  13,
    DMA1_Stream3           =  14,
    DMA1_Stream4           =  15,
    DMA1_Stream5           =  16,
    DMA1_Stream6           =  17,
    DMA1_Stream7           =  47,
    DMA2_Stream0           =  56,
    DMA2_Stream1           =  57,
    DMA2_Stream2           =  58,
    DMA2_Stream3           =  59,
    DMA2_Stream4           =  60,
    DMA2_Stream5           =  68,
    DMA2_Stream6           =  69,
    DMA2_Stream7           =  70,
    EXTI0                  =   6,
    EXTI1                  =   7,
    EXTI2                  =   8,
    EXTI3                  =   9,
    EXTI4                  =  10,
    EXTI9_5                =  23,
    EXTI15_10              =  40,
    FLASH                  =   4,
    FPU                    =  81,
    FSMC                   =  48,
    I2C1_ER                =  32,
    I2C1_EV                =  31,
    I2C2_ER                =  34,
    I2C2_EV                =  33,
    I2C3_ER                =  73,
    I2C3_EV                =  72,
    LP_Timer1              =  93,
    OTG_FS                 =  67,
    OTG_FS_WKUP            =  42,
    OTG_HS                 =  77,
    OTG_HS_EP1_IN          =  75,
    OTG_HS_EP1_OUT         =  74,
    OTG_HS_WKUP            =  76,
    PVD                    =   1,
    QuadSPI                =  92,
    RCC                    =   5,
    RNG                    =  80,
    RTC_ALARM              =  41,
    RTC_WKUP               =   3,
    SAI1                   =  87,
    SAI2                   =  91,
    SDMMC1                 =  49,
    SDMMC2                 = 103,
    SPI1                   =  35,
    SPI2                   =  36,
    SPI3                   =  51,
    SPI4                   =  84,
    SPI5                   =  85,
    TAMP_STAMP             =   2,
    TIM1_BRK_TIM9          =  24,
    TIM1_CC                =  27,
    TIM1_TRG_COM_TIM11     =  26,
    TIM1_UP_TIM10          =  25,
    TIM2                   =  28,
    TIM3                   =  29,
    TIM4                   =  30,
    TIM5                   =  50,
    TIM6_DAC               =  54,
    TIM7                   =  55,
    TIM8_BRK_TIM12         =  43,
    TIM8_CC                =  46,
    TIM8_TRG_COM_TIM14     =  45,
    TIM8_UP_TIM13          =  44,
    UART4                  =  52,
    UART5                  =  53,
    UART7                  =  82,
    UART8                  =  83,
    USART1                 =  37,
    USART2                 =  38,
    USART3                 =  39,
    USART6                 =  71,
    WWDG                   =   0,
    limit = 104,
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
    AHB1ENR  = 0x30,
    AHB2ENR  = 0x34,
    AHB3ENR  = 0x38,
    APB1ENR  = 0x40,
    APB2ENR  = 0x44,
    //CG]
};

namespace ena {
    enum : uint16_t {
        //CG[ svd enables
        ADC1          =  8 + 8 * APB2ENR,
        ADC2          =  9 + 8 * APB2ENR,
        ADC3          = 10 + 8 * APB2ENR,
        AES           =  4 + 8 * AHB2ENR,
        BKPSRAM       = 18 + 8 * AHB1ENR,
        CAN1          = 25 + 8 * APB1ENR,
        CRC           = 12 + 8 * AHB1ENR,
        DAC           = 29 + 8 * APB1ENR,
        DMA1          = 21 + 8 * AHB1ENR,
        DMA2          = 22 + 8 * AHB1ENR,
        DTCMRAM       = 20 + 8 * AHB1ENR,
        FMC           =  0 + 8 * AHB3ENR,
        GPIOA         =  0 + 8 * AHB1ENR,
        GPIOB         =  1 + 8 * AHB1ENR,
        GPIOC         =  2 + 8 * AHB1ENR,
        GPIOD         =  3 + 8 * AHB1ENR,
        GPIOE         =  4 + 8 * AHB1ENR,
        GPIOF         =  5 + 8 * AHB1ENR,
        GPIOG         =  6 + 8 * AHB1ENR,
        GPIOH         =  7 + 8 * AHB1ENR,
        GPIOI         =  8 + 8 * AHB1ENR,
        I2C1          = 21 + 8 * APB1ENR,
        I2C2          = 22 + 8 * APB1ENR,
        I2C3          = 23 + 8 * APB1ENR,
        LPTIM1        =  9 + 8 * APB1ENR,
        OTGFS         =  7 + 8 * AHB2ENR,
        OTGHS         = 29 + 8 * AHB1ENR,
        OTGHSULPI     = 30 + 8 * AHB1ENR,
        PWR           = 28 + 8 * APB1ENR,
        QSPI          =  1 + 8 * AHB3ENR,
        RNG           =  6 + 8 * AHB2ENR,
        RTCAPB        = 10 + 8 * APB1ENR,
        SAI1          = 22 + 8 * APB2ENR,
        SAI2          = 23 + 8 * APB2ENR,
        SDMMC1        = 11 + 8 * APB2ENR,
        SDMMC2        =  7 + 8 * APB2ENR,
        SPI1          = 12 + 8 * APB2ENR,
        SPI2          = 14 + 8 * APB1ENR,
        SPI3          = 15 + 8 * APB1ENR,
        SPI4          = 13 + 8 * APB2ENR,
        SPI5          = 20 + 8 * APB2ENR,
        SYSCFG        = 14 + 8 * APB2ENR,
        TIM1          =  0 + 8 * APB2ENR,
        TIM10         = 17 + 8 * APB2ENR,
        TIM11         = 18 + 8 * APB2ENR,
        TIM12         =  6 + 8 * APB1ENR,
        TIM13         =  7 + 8 * APB1ENR,
        TIM14         =  8 + 8 * APB1ENR,
        TIM2          =  0 + 8 * APB1ENR,
        TIM3          =  1 + 8 * APB1ENR,
        TIM4          =  2 + 8 * APB1ENR,
        TIM5          =  3 + 8 * APB1ENR,
        TIM6          =  4 + 8 * APB1ENR,
        TIM7          =  5 + 8 * APB1ENR,
        TIM8          =  1 + 8 * APB2ENR,
        TIM9          = 16 + 8 * APB2ENR,
        UART4         = 19 + 8 * APB1ENR,
        UART5         = 20 + 8 * APB1ENR,
        UART7         = 30 + 8 * APB1ENR,
        UART8         = 31 + 8 * APB1ENR,
        USART1        =  4 + 8 * APB2ENR,
        USART2        = 17 + 8 * APB1ENR,
        USART3        = 18 + 8 * APB1ENR,
        USART6        =  5 + 8 * APB2ENR,
        USBPHYC       = 31 + 8 * APB2ENR,
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
