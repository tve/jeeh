void Reset_Handler () {
    ((void (*)()) (*(int*) 0x20000204)) (); // jump to new reset vector
}
