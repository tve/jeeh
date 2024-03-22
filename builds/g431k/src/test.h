namespace jeeh {
    void fail (char const* f, int n) {
        constexpr auto N = 30;
        char buf [N+20];
        if (strlen(f) > N)
            f += strlen(f) - N;

        auto itoa = [](char* p, int n) {
            auto i = 1;
            do
                i *= 10;
            while (n >= i);
            do {
                i /= 10;
                *p++ = n / i + '0';
                n %= i;
            } while (i > 1);
            return p;
        };

        strcpy(buf, "failed at ");
        strcat(buf, f);
        auto p = buf + strlen(buf);
        *p++ = ':';
        p = itoa(p, n);
        *p++ = '\n';
        itmWrite(buf, p - buf);

#if MUST_FAIL
        itmWrite("OK\n", 3);
#else
        itmWrite("FAIL\n", 5);
#endif
        while (true) {}
    }

    void hardFaultHandler (uint32_t* sp) {
        enum { CFSR=0x28, HFSR=0x2C, MMAR=0x34, BFAR=0x38 };

        uint32_t hfsr = SCB[HFSR], cfsr = SCB[CFSR],
                bfar = SCB[BFAR], mmar = SCB[MMAR];

        asm ("cpsid i"); // disable all interrupts

        logf("\n[Hard Fault]  SP=%08x  HFSR=%08x  CFSR=%08x", sp, hfsr, cfsr);
        if (hfsr & (1<<30)) {
            if (cfsr & 0xFFFF0000)
                logf("  Usage fault %04x", cfsr >> 16);
            if (cfsr & 0xFF00) {
                logf("  Bus fault %02x", (uint8_t) (cfsr >> 8));
                if (cfsr & (1<<15))
                    logf("    BFAR %08x", bfar);
            }
            if (cfsr & 0xFF) {
                logf("  Memory fault %02x", (uint8_t) cfsr);
                if (cfsr & (1<<7))
                    logf("    MMAR %08x", mmar);
            }
        }

        logf("\t R0=%08x  R1=%08x  R2=%08x  R3=%08x",
                sp[0], sp[1], sp[2], sp[3]);
        logf("\tR12=%08x  LR=%08x  PC=%08x PSR=%08x",
                sp[4], sp[5], sp[6], sp[7]);

        fail();
    }
} // namespace jeeh

struct Tester {
    Tester () {
        extern uint32_t g_pfnVectors [];
        *(uint32_t**) 0xE000'ED08 = g_pfnVectors; // fix SCB->VTOR if in RAM

        //for (auto i = 0; i < 100'000; ++i) asm ("");

        itmWrite("TEST\n", 5);
    }

    ~Tester () {
#if MUST_FAIL
        itmWrite("Should have failed\n", 19);
        itmWrite("FAIL\n", 5);
#else
        itmWrite("OK\n", 3);
#endif
    }
};
