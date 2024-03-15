Debug with SWO output in 4 different terminal windows:

Step | Description | Command | Notes
----:|-------------|---------|-------
1. | start a continuous debug build: | **`make debug`** | |
2. | start the `gdb` debugger: | **`make gdb`** | |
3. | start the serial console: | **`make console`** | optional
4. | start SWO log output: | **`make logs`** | |
5. | in gdb (step 2), type: | **`c`** | 

To update: edit/save changes, check 1) messages, then in gdb: **`^C`** +
**`load`** + **`r`**.  
_(Log output is only produced when gdb is used - unplug/replug board to
disable!)_

To only build, upload, and run the serial console: **`make upload`** + **`make
console`**.

The Nucleo-G431KB includes an ST-Link V3E. When the CPU runs at 150
MHz, log output can reliably be sent at 9.375 MBaud, i.e. roughly 1
µs/byte (whereas special setup would be needed for 170 MHz).
