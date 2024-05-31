namespace jeeh {
    void fail (void const* a, char const* f, int n) {
        logf("\n" "failed at %s:%d\n"
                  "failed caller: %p", f, n, a);
#if MUST_FAIL
        logWriter("OK\n", 3);
#else
        logWriter("FAIL\n", 5);
#endif
        while (true) {}
    }

} // namespace jeeh

struct Tester {
    Tester () {
        extern uint32_t g_pfnVectors [];
        *(uint32_t**) 0xE000'ED08 = g_pfnVectors; // fix SCB->VTOR if in RAM

        fastClock();
#if SWO_FREQ
        swoInit(SWO_FREQ); // TODO openocd didn't init ITM/SWO on STM32WL
#endif
        swoWrite("\nTEST\n", 6);
    }

    ~Tester () {
#if MUST_FAIL
        logf("Should have failed!\nFAIL");
#else
        logf("OK", 3);
#endif
    }
};
