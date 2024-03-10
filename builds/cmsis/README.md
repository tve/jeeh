# Blink an LED

This demo is for a
[Nucleo-G431KB](https://www.st.com/en/evaluation-tools/nucleo-g431kb.html) board
with its LED on pin PB8.  The code uses only
[CMSIS](https://github.com/ARM-software/CMSIS_5), it does _not_ use JeeH.  It
can be used to to verify the [PlatformIO](https://platformio.org/install/cli)
installation and the Nucleo board connection.

The following command builds and uploads the demo: **`pio run -e main -t
upload`**

There's an alternative method to upload and run code without re-flashing:

1. Upload a "boot stub" to flash memory: **`pio run -e stub -t upload`**
2. Upload the RAM-based build: **`pio run -e ram -t upload`**
3. There is no step 3. Repeat step 2 after power-up and for each new build.

This is faster and avoids wearing out flash memory.  It needs the `st-flash` utility to
upload to RAM.  See <https://github.com/stlink-org/stlink> (e.g. **`brew install
stlink`** on MacOS).
