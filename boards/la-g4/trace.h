// This header can be included in JeeH to visualise some key call events.

enum Trace {
    /* 0 */ INIT,
    /* 1 */ SEND,
    /* 2 */ RECV,
    /* 3 */ TICKED,
    /* 4 */ IRQDISP,
    /* 5 */ TRIGGER,
    /* 6 */ SUBMIT,
    /* 7 */ RESCHED,
};

extern Pin tracePins [];

template< int N >
struct Tracer {
    Tracer () { tracePins[N] = 1; }
    ~Tracer () { tracePins[N] = 0; }
};

#define trace(x) jeeh::Tracer<Trace::x> _tr; // create object in current scope
