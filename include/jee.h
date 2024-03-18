// This is the top-level header file for JeeH.
// Lines with "CG" control the code-generated parts of this file.

#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>

//CG3 svd defines
#define STM32   1
#define STM32G4 1
#define SVDNAME "STM32G431xx"

#ifdef NASSERT
#define assert(x) ((void) 0) // don't evaluate x (i.e. prevent side-effects)
#else
#define assert(x) do if (!(x)) jeeh::fail(); while (false) // see jee-sys.h
#endif

namespace jeeh {

//CG1 version
constexpr auto VERSION = "v6.0a1-13-g04f0960";

#include "arch/sys.h"

#if STM32
#include "arch/ioreg.h"
#include "jee-stm32.h"
#include "arch/pin.h"
#endif

} // namespace jeeh

extern uint32_t SystemCoreClock; // Hz, set in CMSIS startup
