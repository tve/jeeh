## Summary of tests

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
`t17` | date   | Exercise the DateTime type and optionally test setting it from the current compile date and time.
`t18` | rtc    | Test the `DateTime` class and the Real-Time Clock (the Nucleo-32's STM32G431KB can only run its RTC off the LSI clock).
`t19` | dog    | Try out the watchdog, report the reset cause, and let the watchdog expire to reset the system.
`t20` | align  | Show the sizes of several core data types and their alignment requirements.
`t21` | nest   | A minimal example of nested tasks, used to weed out a nasty bug in sys::recv.
`t22` | clock  | Use the real-time clock to enter stop mode and resume through a wakeup event (from 1 ms to 16s).

There are also some board-specific tests in `tests/basic/onboard-*.cpp`.
