// Hardware random number generator.

#if !(STM32F1 | STM32F3 | STM32L0)
namespace jeeh::rng {

enum { CRRCR=0x98 };

void init () {
#if !STM32F4
    RCC[CRRCR](0) = 1;        // HSI48ON
    while (!RCC[CRRCR](1)) {} // ~HSI48RDY
#endif

    RCC(ena::RNG,1) = 1;
    RNG[0x00](2) = 1; // RNGEN
}

void deinit () {
    RCC(ena::RNG,1) = 0;
}

// return a 32-bit random number (but never zero)
uint32_t rand () {
    uint32_t r;
    do
        r = RNG[0x08];
    while (r == 0);
    return r;
}

// see https://en.wikipedia.org/wiki/Fisher–Yates_shuffle
template< int N >
struct Permutation {
    static_assert(1 <= N && N <= 65536);
    uint16_t choice [N], limit;

    void init () {
        rng::init();
        for (auto i = 0; i < N; ++i)
            choice[i] = i;
        limit = N;
    }

    int next () {
        if (limit <= 0)
            return -1;
        auto i = rand() % limit;
        auto r = choice[i];
        choice[i] = choice[--limit];
        return r;
    }

    void shuffle () {
        limit = N;
        for (auto i = 0; i < N; ++i) {
            auto r = next(); // decrements limit
            choice[limit] = r;
        }
        assert(limit == 0);
    }
};

} // namespace jeeh::rng
#endif // !(STM32F1 | STM32L0)
