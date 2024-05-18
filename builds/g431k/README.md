**This is a set of small tests for use with the Nucleo-32 G431KB board.**

> Note: some tests expect a jumper between D0 and D1 (GPIO PA9 and PA10).

For a description of tests, see the README file in `builds/`.

## RAM-based uploads

To improve test throughput, the builds are RAM-based. Uploads to RAM are just a
bit faster than erasing and rewriting flash memory, which helps when running
many tests in quick succession. It also avoids wear and tear of the on-board
flash.  For this to work, a minimal boot "stub" has to be stored in flash memory
(just once), as follows:

    make stub

## Continuous testing

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

## Test output via ITM/SWO

Another property of these tests, is that they use the ITM (Instruction Trace
Macrocell) to generate output through the SWO pin which is connected to the
on-board ST-Link V3. The UART tied to the ST-Link's `VCP` endpoint is not
necessary to verify tests and can be used to develop the UART driver itself, or
to implement an app console, for example.  Although ITM is output-only and
polled, with an ST-Link V3 it can be very fast (over 10 Mbaud when the µC is
running at 170 MHz - this was one reason to pick this specific board).

## Code generator

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
