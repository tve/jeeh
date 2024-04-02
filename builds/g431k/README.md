**This is a set of small tests for use with the Nucleo-32 G431KB board.**

> Note: some of these tests expect a jumper between D0 and D1 (GPIO PA9 and
> PA10).

### Summary of tests

In increasing order of complexity, more or less:

Test  | Name   | Description
------|--------|------------
`t00` | itm    | This is a minimal build to verify the test mechanism, by reporting the lines `TEST` and `OK`. It redefines `jeeh::fail()` to reduce the code footprint.
`t01` | tester | This verifies that the boilerplate code used in these tests is functional. It merely instantiates the `Tester` object defined in `src/test.h`.
`t02` | dump   | A quick check that many lines of test output can be sent back-to-back, and that all of it comes out and gets captured.
`t03` | led    | Test the use of GPIO pins and JeeH's `Pin` class. This turns the on-board LED on and then exits.
`t04` | jumper | Verify that the jumper between `PA9` and `PA10` is present.  This can then be used to test EXTI interrupts and a UART in loopback mode.
`t05` | serio  | Implement a crude polled UART and send some text through its FIFO's to verify that an entire (short) message can pass through correctly.
`t06` | logf   | Test the `logf()` function in JeeH, which is like printf, but sends it output over ITM/SWO (and drops it when no debugger is present).
`t07` | fault  | Generate a fault exception (a "usage fault" in this case) to verify that it is caught and reported by the `hardFaultHandler` set up by the `Tester` object.
`t08` | pool   | Try out the memory allocator via the `sys::pool()` system call interface (which makes it thread-safe).
`t09` | wait   | Perform a brief delay through the `sys::wait()` system call, which sets up `SysTick` interrupts and implements a message-based timer chain.
`t10` | task   | Create a very simple `Doubler` task, and use JeeH's message-based `sys::call()` mechanism to pass information into it and back.
`t11` | block  | Use a blocking `sys::wait()` call inside a task. This needs to be special-cased to suspend and resume the task's owner thread.
`t12` | thread | Fork a second thread and let them each block in an alternating manner to exercise context switching, then wait on thread exit.
`t13` | uart   | Test the DMA-based UART driver by sending out text over the loopback-jumper at maximum speed and counting the bytes being received.
`t14` | exti   | Test the EXTI driver, i.e. blocking until a pin-change interrupt comes back after toggling an output jumpered to an input.
`t15` | cycles | Access the CPU cycle counter present in all ARM Cortex families except M0 and M0+.
`t16` | philo  | An implementation of the "dining philosophers problem", using threads and the `Lock` type to synchronize their resource access.
`t17` | rtc    | Test the `DateTime` class and the Real-Time Clock (the Nucleo-32's STM32G431KB can only run its RTC off the LSI clock).
`t18` | dog    | Try out the watchdog, report the reset cause, and let the watchdog expire to reset the system.
`t19` | align  | Show the sizes of several core data types and their alignment requirements.

### RAM-based uploads

To improve test throughput, the builds are RAM-based. Uploads to RAM are just a
bit faster than erasing and rewriting flash memory, which helps when running
many tests in quick succession. It also avoids wear and tear of the on-board
flash.  For this to work, a minimal boot "stub" has to be stored in flash memory
(just once), as follows:

    cd stub; pio run -t upload

### Continuous testing

The full set of tests can be launched by typing `make` or `make all`. This will
launch a continuous test loop, which triggers whenever a source file change is
detected. This needs a small utility called `entr`. To install this:

- on MacOS: `brew install entr`
- on Linux: `apt install entr`

The whole set of tests still takes several seconds, but there is also a way to
run (and continuously re-run) a single test, e.g.

    make check E=t03

All test output is saved in separate files in the `log/` subdirectory. When a
test passes normally (i.e. when it ends with the "OK" output message), only the
name of the test is shown. Else, the full test output is also shown on-screen.

### Test output via ITM/SWO

Another property of these tests, is that they use the ITM (Instruction Trace
Macrocell) to generate output through the SWO pin which is connected to the
on-board ST-Link V3. The UART tied to the ST-Link's `VCP` endpoint is not
necessary to verify tests and can be used to develop the UART driver itself, or
to implement an app console, for example.  Although ITM is output-only and
polled, with an ST-Link V3 it can be very fast (about 10 Mbaud when the µC is
running at 150 MHz - this was one reason to pick this specific board).

### Code generator

JeeH includes a code generator (see `make/codegen.py`) to generate a few of its
own headers on the fly, based on the target CPU (STM32G4 in this case).  This
code generator is driven by comment lines in headers which start with `//CG`.
The code generator can also be used in application code, such as these tests. It
will scan all headers in the source directory (in this case the default `src/`
area). The `src/defs.h` file uses this to extract settings from `platformio.ini`
(`build_leds = ...` and a few more). The benefit of such an approach is that
different highly project-specific features can be specified in separate
PlatformIO `[env:...]` sections, without having to use lots of `#ifdef` lines
and `-D...` flags. New functionality can be added in `make/cgdefs.py` to add
more custom features. This code generation is bound to evolve a lot further as
the development of JeeH and these tests progress.

There are many conveniences when using a "source-modifying" code generator
design like this, but there is also a downside: in projects which build for more
than one CPU type, the generated files will differ, based on whichever build
came last.  There is a "strip" option in the code generator, where it removes
all the lines that can be reproduced again later on. This can be done with `make
strip`.

_Work in progress ..._
