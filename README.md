# JeeH

JeeH is a runtime library for STM32 microcontrollers. It offers an alternative
to the ones included with the Arduino IDE, STM32CubeMX, and others.

**Features**

- Based on messages for synchronisarion and (optionally) multi-threaded.
- Written in C++17 with limited use of templates and not dependent on STL.
- The build environment is PlatformIO, for either command-line or IDE use.
- Compiled code relies on the CMSIS framework's startup and linker files.
- JeeH is _very_ Lean and Mean. All its source code is in the public domain.

**Status**

JeeH is in active development. All the API and naming conventions can change.  
Latest changes are in the `dev` branch of <https://git.sr.ht/~jcw/jeeh/refs>.  
Issues & bugs are (not actively) tracked at <https://todo.sr.ht/~jcw/issues>.  
Documentation? Yeah, some day. For now just <https://jc.wippler.nl/docs/jeeh/>.
