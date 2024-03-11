// This is the top-level header file for JeeH.
// Lines with "CG" control the code-generated parts of this file.

#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>

//CG: svd defines

namespace jeeh {

//CG1 version
constexpr auto VERSION = "<stripped>";

#if STM32
#include "arch/ioreg.h"
#include "jee-stm32.h"
#include "arch/pin.h"
#endif

} // namespace jeeh

extern uint32_t SystemCoreClock; // Hz, set in CMSIS startup
