# JeeH w/ and w/o Arduino runtime

This demo blinks the on-board led a few times, and prints some messages to the
serial port.  
Then an assertion failure is forced, just to verify that this works.

With (all IRQs handled by the Arduino runtime code):

    pio run -t upload -e l432k-arduino

Without (all IRQs handled by the JeeH runtime code):

    pio run -t upload -e l432k-cmsis
