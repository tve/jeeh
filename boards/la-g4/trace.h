// This header can be included in JeeH to visualise some key call events.

//CG[ board trace
#define TRACE_I "A12"
#define TRACE_S "B0"
#define TRACE_R "B6"
#define TRACE_D "A7"
#define TRACE_P "A15"
#define TRACE_X "B7"
#define TRACE_Y "A4"
#define TRACE_Z "A1"
#define TRACE_0 "A0"
#define TRACE_1 "A8"
#define TRACE_2 "A11"
#define TRACE_3 "B3"
#define TRACE_4 "B5"
#define TRACE_5 "B4"
//CG]

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
