void Reset_Handler () {
#if STM32H7 // 0x20000000 is in DTCM - can't be used for reset/irq vectors?
    (*(void (**)()) 0x24000004) ();  // jump to new reset vector in RAM
#else
    (*(void (**)()) 0x20000204) ();  // jump to new reset vector in RAM
#endif
}
