#include "cycles.h"
#include "exti.h"
#include "i2c.h"
#include "spi.h"

#if STM32F1 | STM32F3 | STM32F4 | STM32F7 | STM32G4 | STM32H7
#include "uart-f1347g4h7.h"
#elif STM32L0 | STM32L4 | STM32WL
#include "uart-l04wl.h"
#endif
