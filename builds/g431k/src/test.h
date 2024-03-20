namespace jeeh {
    void fail (char const* f, int n) {
        constexpr auto N = 60;
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

        strcpy(buf, f);
        auto p = buf + strlen(buf);
        *p++ = ':';
        p = itoa(p, n);
        strcpy(p, " failed\n");
        itmWrite(buf, strlen(buf));

        itmWrite("FAIL\n", 5);
        while (true) {}
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
        itmWrite("OK\n", 3);
    }
};
