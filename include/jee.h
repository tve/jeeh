// This is the top-level header file for JeeH.
// Lines with "CG" control the code-generated parts of this file.

#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>

//CG: svd defines

#ifdef NASSERT
#define assert(x) ((void) 0) // don't evaluate x (i.e. prevent side-effects)
#else
// see https://interrupt.memfault.com/blog/asserts-in-embedded-systems
#define assert(x) do if (!(x)) jeeh::fail(); while (false) // see jee-sys.h
#endif

namespace jeeh {

//CG1 version
constexpr auto VERSION = "<stripped>";

#include "arch/sys.h"

#if STM32
#include "arch/ioreg.h"
#include "jee-stm32.h"
#include "arch/pin.h"
#endif

} // namespace jeeh

extern "C" uint32_t SystemCoreClock; // Hz, set in CMSIS startup
