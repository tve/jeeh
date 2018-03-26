#define main myMain
extern int myMain ();

#define wait_ms delay

extern "C" {
    void pinMode (uint8_t,uint8_t);
    int digitalRead (uint8_t);
    void digitalWrite (uint8_t,uint8_t);
    void delay (unsigned long);
}

enum class Pinmode {
    in_float  = 0,
    in_pullup = 2,
    out       = 1,
    out_od    = 2,
};

template<char port,int pin>
struct Pin {
    constexpr static int off = port == 'B' ?  8 :  // PB 0..7
                               port == 'C' ? 14 :  // PC 0..7
                                              0;   // else PD 0..7
    constexpr static int id = off + pin;

    static void mode (Pinmode m) {
        pinMode(id, (int) m);
    }

    static int read () {
        return digitalRead(id);
    }

    static void write (int v) {
        digitalWrite(id, v);
    }

    // shorthand
    operator int () const { return read(); }
    void operator= (int v) const { write(v); }

    static void toggle () {
        write(!read());
    }
};
