**Drive a 480x320 LCD display in 8-bit parallel mode with Nucleo-64 G474.**

The drawback is that those 8 bits are mixed up across different GPIO ports.  
In normal mode (using a loop over 8 pins), this takes 430 ms.  
In "fast" mode (with a special-case BSRR), this takes 13 ms.

Sample output:

    lcd: STM32G431xx @ 160 MHz
    init 130021 us
    clear 12487 us
    fill    630 us
    pixel     6 us
