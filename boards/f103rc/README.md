# Comms test board

This uses an STM32F103RC, which has 5 UARTs. The first 4 support DMA, the last
one (UART5) is used as TX-only console (PD2 is tied to the on-board LED).

Similar: <https://stm32-base.org/boards/STM32F103RCT6-Generic-Board>.

Peripheral | TX pin | RX pin
-----------|--------|-------
USART1     | PA9    | PA10
USART2     | PA2    | PA3
USART3     | PB10   | PB11
UART4      | PC10   | PC11
