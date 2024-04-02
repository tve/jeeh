// Definitions for STM32 microcontrollers (see also src/stm32.cpp).
// Lines with "CG" control the code-generated parts of this file.

//CG[ svd ioregs
constexpr IoReg<0xE000'EF90> AC;
constexpr IoReg<0x4002'2000> ADC1;
constexpr IoReg<0x4002'2100> ADC2;
constexpr IoReg<0x5802'6000> ADC3;
constexpr IoReg<0x5802'6300> ADC3_COMMON;
constexpr IoReg<0x4002'2300> ADC12_COMMON;
constexpr IoReg<0x5100'0000> AXI;
constexpr IoReg<0x5802'5400> BDMA;
constexpr IoReg<0x4000'A800> CAN_CCU;
constexpr IoReg<0x4000'6C00> CEC;
constexpr IoReg<0x5800'3800> COMP1;
constexpr IoReg<0x5802'4C00> CRC;
constexpr IoReg<0x4000'8400> CRS;
constexpr IoReg<0x4000'7400> DAC;
constexpr IoReg<0x5C00'1000> DBGMCU;
constexpr IoReg<0x4802'0000> DCMI;
constexpr IoReg<0x5200'6000> DELAY_BLOCK_QUADSPI;
constexpr IoReg<0x5200'8000> DELAY_BLOCK_SDMMC1;
constexpr IoReg<0x4802'2800> DELAY_BLOCK_SDMMC2;
constexpr IoReg<0x4001'7000> DFSDM;
constexpr IoReg<0x4002'0000> DMA1;
constexpr IoReg<0x4002'0400> DMA2;
constexpr IoReg<0x5200'1000> DMA2D;
constexpr IoReg<0x4002'0800> DMAMUX1;
constexpr IoReg<0x5802'5800> DMAMUX2;
constexpr IoReg<0x4002'9000> ETHERNET_DMA;
constexpr IoReg<0x4002'8000> ETHERNET_MAC;
constexpr IoReg<0x4002'8C00> ETHERNET_MTL;
constexpr IoReg<0x5800'0000> EXTI;
constexpr IoReg<0x4000'A000> FDCAN1;
constexpr IoReg<0x4000'A400> FDCAN2;
constexpr IoReg<0x5200'2000> FLASH;
constexpr IoReg<0x5200'4000> FMC;
constexpr IoReg<0xE000'EF34> FPU;
constexpr IoReg<0xE000'ED88> FPU_CPACR;
constexpr IoReg<0x5802'0000> GPIOA;
constexpr IoReg<0x5802'0400> GPIOB;
constexpr IoReg<0x5802'0800> GPIOC;
constexpr IoReg<0x5802'0C00> GPIOD;
constexpr IoReg<0x5802'1000> GPIOE;
constexpr IoReg<0x5802'1400> GPIOF;
constexpr IoReg<0x5802'1800> GPIOG;
constexpr IoReg<0x5802'1C00> GPIOH;
constexpr IoReg<0x5802'2000> GPIOI;
constexpr IoReg<0x5802'2400> GPIOJ;
constexpr IoReg<0x5802'2800> GPIOK;
constexpr IoReg<0x4001'7780> HRTIM_COMMON;
constexpr IoReg<0x4001'7400> HRTIM_MASTER;
constexpr IoReg<0x4001'7480> HRTIM_TIMA;
constexpr IoReg<0x4001'7500> HRTIM_TIMB;
constexpr IoReg<0x4001'7580> HRTIM_TIMC;
constexpr IoReg<0x4001'7600> HRTIM_TIMD;
constexpr IoReg<0x4001'7680> HRTIM_TIME;
constexpr IoReg<0x5802'6400> HSEM;
constexpr IoReg<0x4000'5400> I2C1;
constexpr IoReg<0x4000'5800> I2C2;
constexpr IoReg<0x4000'5C00> I2C3;
constexpr IoReg<0x5800'1C00> I2C4;
constexpr IoReg<0x5800'4800> IWDG;
constexpr IoReg<0x5200'3000> JPEG;
constexpr IoReg<0x4000'2400> LPTIM1;
constexpr IoReg<0x5800'2400> LPTIM2;
constexpr IoReg<0x5800'2800> LPTIM3;
constexpr IoReg<0x5800'2C00> LPTIM4;
constexpr IoReg<0x5800'3000> LPTIM5;
constexpr IoReg<0x5800'0C00> LPUART1;
constexpr IoReg<0x5000'1000> LTDC;
constexpr IoReg<0x4000'9400> MDIOS;
constexpr IoReg<0x5200'0000> MDMA;
constexpr IoReg<0xE000'ED90> MPU;
constexpr IoReg<0xE000'E100> NVIC;
constexpr IoReg<0xE000'EF00> NVIC_STIR;
constexpr IoReg<0x4000'9000> OPAMP;
constexpr IoReg<0x4004'0800> OTG1_HS_DEVICE;
constexpr IoReg<0x4004'0000> OTG1_HS_GLOBAL;
constexpr IoReg<0x4004'0400> OTG1_HS_HOST;
constexpr IoReg<0x4004'0E00> OTG1_HS_PWRCLK;
constexpr IoReg<0x4008'0800> OTG2_HS_DEVICE;
constexpr IoReg<0x4008'0000> OTG2_HS_GLOBAL;
constexpr IoReg<0x4008'0400> OTG2_HS_HOST;
constexpr IoReg<0x4008'0E00> OTG2_HS_PWRCLK;
constexpr IoReg<0xE000'ED78> PF;
constexpr IoReg<0x5802'4800> PWR;
constexpr IoReg<0x5200'5000> QUADSPI;
constexpr IoReg<0x5802'4400> RCC;
constexpr IoReg<0x4802'1800> RNG;
constexpr IoReg<0x5800'4000> RTC;
constexpr IoReg<0x4001'5800> SAI1;
constexpr IoReg<0x4001'5C00> SAI2;
constexpr IoReg<0x4001'6000> SAI3;
constexpr IoReg<0x5800'5400> SAI4;
constexpr IoReg<0xE000'ED00> SCB;
constexpr IoReg<0xE000'E008> SCB_ACTRL;
constexpr IoReg<0x5200'7000> SDMMC1;
constexpr IoReg<0x4802'2400> SDMMC2;
constexpr IoReg<0x4000'4000> SPDIFRX;
constexpr IoReg<0x4001'3000> SPI1;
constexpr IoReg<0x4000'3800> SPI2;
constexpr IoReg<0x4000'3C00> SPI3;
constexpr IoReg<0x4001'3400> SPI4;
constexpr IoReg<0x4001'5000> SPI5;
constexpr IoReg<0x5800'1400> SPI6;
constexpr IoReg<0xE000'E010> STK;
constexpr IoReg<0x4000'8800> SWPMI;
constexpr IoReg<0x5800'0400> SYSCFG;
constexpr IoReg<0x4001'0000> TIM1;
constexpr IoReg<0x4000'0000> TIM2;
constexpr IoReg<0x4000'0400> TIM3;
constexpr IoReg<0x4000'0800> TIM4;
constexpr IoReg<0x4000'0C00> TIM5;
constexpr IoReg<0x4000'1000> TIM6;
constexpr IoReg<0x4000'1400> TIM7;
constexpr IoReg<0x4001'0400> TIM8;
constexpr IoReg<0x4000'1800> TIM12;
constexpr IoReg<0x4000'1C00> TIM13;
constexpr IoReg<0x4000'2000> TIM14;
constexpr IoReg<0x4001'4000> TIM15;
constexpr IoReg<0x4001'4400> TIM16;
constexpr IoReg<0x4001'4800> TIM17;
constexpr IoReg<0x4000'4C00> UART4;
constexpr IoReg<0x4000'5000> UART5;
constexpr IoReg<0x4000'7800> UART7;
constexpr IoReg<0x4000'7C00> UART8;
constexpr IoReg<0x4001'1000> USART1;
constexpr IoReg<0x4000'4400> USART2;
constexpr IoReg<0x4000'4800> USART3;
constexpr IoReg<0x4001'1400> USART6;
constexpr IoReg<0x5800'3C00> VREFBUF;
constexpr IoReg<0x5000'3000> WWDG;
//CG]

enum struct Irq : uint8_t {
    //CG[ svd irqs
    ADC1_2                 =  18,
    ADC3                   = 127,
    BDMA_CH1               = 129,
    BDMA_CH2               = 130,
    BDMA_CH3               = 131,
    BDMA_CH4               = 132,
    BDMA_CH5               = 133,
    BDMA_CH6               = 134,
    BDMA_CH7               = 135,
    BDMA_CH8               = 136,
    CEC                    =  94,
    COMP                   = 137,
    CRS                    = 144,
    DCMI                   =  78,
    DFSDM1_FLT0            = 110,
    DFSDM1_FLT1            = 111,
    DFSDM1_FLT2            = 112,
    DFSDM1_FLT3            = 113,
    DMA1_STR0              =  11,
    DMA1_STR1              =  12,
    DMA1_STR2              =  13,
    DMA1_STR3              =  14,
    DMA1_STR4              =  15,
    DMA1_STR5              =  16,
    DMA1_STR6              =  17,
    DMA1_STR7              =  47,
    DMA2_STR0              =  56,
    DMA2_STR1              =  57,
    DMA2_STR2              =  58,
    DMA2_STR3              =  59,
    DMA2_STR4              =  60,
    DMA2_STR5              =  68,
    DMA2_STR6              =  69,
    DMA2_STR7              =  70,
    DMA2D                  =  90,
    DMAMUX1_OV             = 102,
    DMAMUX2_OVR            = 128,
    ETH                    =  61,
    ETH_WKUP               =  62,
    EXTI0                  =   6,
    EXTI1                  =   7,
    EXTI2                  =   8,
    EXTI3                  =   9,
    EXTI4                  =  10,
    EXTI9_5                =  23,
    EXTI15_10              =  40,
    FDCAN1_IT0             =  19,
    FDCAN1_IT1             =  21,
    FDCAN2_IT0             =  20,
    FDCAN2_IT1             =  22,
    FDCAN_CAL              =  63,
    FLASH                  =   4,
    FMC                    =  48,
    FPU                    =  81,
    HRTIM1_FLT             = 109,
    HRTIM1_MST             = 103,
    HRTIM1_TIMA            = 104,
    HRTIM1_TIMC            = 106,
    HRTIM1_TIMD            = 107,
    HRTIM_TIMB             = 105,
    HRTIM_TIME             = 108,
    HSEM0                  = 125,
    I2C1_ER                =  32,
    I2C1_EV                =  31,
    I2C2_ER                =  34,
    I2C2_EV                =  33,
    I2C3_ER                =  73,
    I2C3_EV                =  72,
    I2C4_ER                =  96,
    I2C4_EV                =  95,
    JPEG                   = 121,
    LPTIM1                 =  93,
    LPTIM2                 = 138,
    LPTIM3                 = 139,
    LPTIM4                 = 140,
    LPTIM5                 = 141,
    LPUART                 = 142,
    LTDC                   =  88,
    LTDC_ER                =  89,
    MDIOS                  = 120,
    MDIOS_WKUP             = 119,
    MDMA                   = 122,
    OTG_FS                 = 101,
    OTG_FS_EP1_IN          =  99,
    OTG_FS_EP1_OUT         =  98,
    OTG_FS_WKUP            = 100,
    OTG_HS                 =  77,
    OTG_HS_EP1_IN          =  75,
    OTG_HS_EP1_OUT         =  74,
    OTG_HS_WKUP            =  76,
    PVD_PVM                =   1,
    QUADSPI                =  92,
    RCC                    =   5,
    RTC_ALARM              =  41,
    RTC_TAMP_STAMP_CSS_LSE =   2,
    RTC_WKUP               =   3,
    SAI1                   =  87,
    SAI2                   =  91,
    SAI3                   = 114,
    SAI4                   = 146,
    SDMMC                  = 124,
    SDMMC1                 =  49,
    SPDIF                  =  97,
    SPI1                   =  35,
    SPI2                   =  36,
    SPI3                   =  51,
    SPI4                   =  84,
    SPI5                   =  85,
    SPI6                   =  86,
    SWPMI1                 = 115,
    TIM1_BRK               =  24,
    TIM1_TRG_COM           =  26,
    TIM1_UP                =  25,
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
    TIM15                  = 116,
    TIM16                  = 117,
    TIM17                  = 118,
    TIM_CC                 =  27,
    UART4                  =  52,
    UART5                  =  53,
    UART7                  =  82,
    UART8                  =  83,
    USART1                 =  37,
    USART2                 =  38,
    USART3                 =  39,
    USART6                 =  71,
    WKUP                   = 149,
    WWDG1                  =   0,
    WWDG1_RST              = 143,
    limit = 150
    //CG]
};

enum : uint16_t {
    //CG[ svd rccs
    AHB1ENR  = 0xD8,
    AHB2ENR  = 0xDC,
    AHB3ENR  = 0xD4,
    AHB4ENR  = 0xE0,
    APB1LENR = 0xE8,
    APB2ENR  = 0xF0,
    APB3ENR  = 0xE4,
    APB4ENR  = 0xF4,
    //CG]
};

namespace ena {
    enum : uint16_t {
        //CG[ svd enables
        ADC12         =  5 + 8 * AHB1ENR,
        ADC3          = 24 + 8 * AHB4ENR,
        BDMA          = 21 + 8 * AHB4ENR,
        BKPRAM        = 28 + 8 * AHB4ENR,
        CAMITF        =  0 + 8 * AHB2ENR,
        CEC           = 27 + 8 * APB1LENR,
        COMP12        = 14 + 8 * APB4ENR,
        CRC           = 19 + 8 * AHB4ENR,
        CRYPT         =  4 + 8 * AHB2ENR,
        DAC12         = 29 + 8 * APB1LENR,
        DFSDM1        = 28 + 8 * APB2ENR,
        DMA1          =  0 + 8 * AHB1ENR,
        DMA2          =  1 + 8 * AHB1ENR,
        DMA2D         =  4 + 8 * AHB3ENR,
        ETH1MAC       = 15 + 8 * AHB1ENR,
        ETH1RX        = 17 + 8 * AHB1ENR,
        ETH1TX        = 16 + 8 * AHB1ENR,
        FMC           = 12 + 8 * AHB3ENR,
        GPIOA         =  0 + 8 * AHB4ENR,
        GPIOB         =  1 + 8 * AHB4ENR,
        GPIOC         =  2 + 8 * AHB4ENR,
        GPIOD         =  3 + 8 * AHB4ENR,
        GPIOE         =  4 + 8 * AHB4ENR,
        GPIOF         =  5 + 8 * AHB4ENR,
        GPIOG         =  6 + 8 * AHB4ENR,
        GPIOH         =  7 + 8 * AHB4ENR,
        GPIOI         =  8 + 8 * AHB4ENR,
        GPIOJ         =  9 + 8 * AHB4ENR,
        GPIOK         = 10 + 8 * AHB4ENR,
        HASH          =  5 + 8 * AHB2ENR,
        HRTIM         = 29 + 8 * APB2ENR,
        HSEM          = 25 + 8 * AHB4ENR,
        I2C1          = 21 + 8 * APB1LENR,
        I2C2          = 22 + 8 * APB1LENR,
        I2C3          = 23 + 8 * APB1LENR,
        I2C4          =  7 + 8 * APB4ENR,
        JPGDEC        =  5 + 8 * AHB3ENR,
        LPTIM1        =  9 + 8 * APB1LENR,
        LPTIM2        =  9 + 8 * APB4ENR,
        LPTIM3        = 10 + 8 * APB4ENR,
        LPTIM4        = 11 + 8 * APB4ENR,
        LPTIM5        = 12 + 8 * APB4ENR,
        LPUART1       =  3 + 8 * APB4ENR,
        LTDC          =  3 + 8 * APB3ENR,
        MDMA          =  0 + 8 * AHB3ENR,
        QSPI          = 14 + 8 * AHB3ENR,
        RNG           =  6 + 8 * AHB2ENR,
        RTCAPB        = 16 + 8 * APB4ENR,
        SAI1          = 22 + 8 * APB2ENR,
        SAI2          = 23 + 8 * APB2ENR,
        SAI3          = 24 + 8 * APB2ENR,
        SAI4          = 21 + 8 * APB4ENR,
        SDMMC1        = 16 + 8 * AHB3ENR,
        SDMMC2        =  9 + 8 * AHB2ENR,
        SPDIFRX       = 16 + 8 * APB1LENR,
        SPI1          = 12 + 8 * APB2ENR,
        SPI2          = 14 + 8 * APB1LENR,
        SPI3          = 15 + 8 * APB1LENR,
        SPI4          = 13 + 8 * APB2ENR,
        SPI5          = 20 + 8 * APB2ENR,
        SPI6          =  5 + 8 * APB4ENR,
        SRAM1         = 29 + 8 * AHB2ENR,
        SRAM2         = 30 + 8 * AHB2ENR,
        SRAM3         = 31 + 8 * AHB2ENR,
        SYSCFG        =  1 + 8 * APB4ENR,
        TIM1          =  0 + 8 * APB2ENR,
        TIM12         =  6 + 8 * APB1LENR,
        TIM13         =  7 + 8 * APB1LENR,
        TIM14         =  8 + 8 * APB1LENR,
        TIM15         = 16 + 8 * APB2ENR,
        TIM16         = 17 + 8 * APB2ENR,
        TIM17         = 18 + 8 * APB2ENR,
        TIM2          =  0 + 8 * APB1LENR,
        TIM3          =  1 + 8 * APB1LENR,
        TIM4          =  2 + 8 * APB1LENR,
        TIM5          =  3 + 8 * APB1LENR,
        TIM6          =  4 + 8 * APB1LENR,
        TIM7          =  5 + 8 * APB1LENR,
        TIM8          =  1 + 8 * APB2ENR,
        UART4         = 19 + 8 * APB1LENR,
        UART5         = 20 + 8 * APB1LENR,
        USART1        =  4 + 8 * APB2ENR,
        USART2        = 17 + 8 * APB1LENR,
        USART3        = 18 + 8 * APB1LENR,
        USART6        =  5 + 8 * APB2ENR,
        USART7        = 30 + 8 * APB1LENR,
        USART8        = 31 + 8 * APB1LENR,
        USB1OTG       = 25 + 8 * AHB1ENR,
        USB1ULPI      = 26 + 8 * AHB1ENR,
        USB2OTG       = 27 + 8 * AHB1ENR,
        USB2OTGHSULPI = 18 + 8 * AHB1ENR,
        USB2ULPI      = 28 + 8 * AHB1ENR,
        VREF          = 15 + 8 * APB4ENR,
        WWDG1         =  6 + 8 * APB3ENR,
        //CG]
#if STM32F3
        GPIOA         = 17 + 8 * 0x14,
#elif STM32G0
        GPIOA         =  0 + 8 * 0x34,
#elif STM32L0
        GPIOA         =  8 * 0x2C,
#endif
    };
} // namespace ena

uint32_t fastClock (bool pll =true);
void itmWrite (void const* ptr, size_t len);
