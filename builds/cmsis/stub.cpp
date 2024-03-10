extern "C"
void Reset_Handler () {
    (*(void (**)()) 0x20000204) ();  // jump to new reset vector in RAM
}
