// Header file for the the central system types and functions.

[[noreturn]]
void fail (void const* addr =__builtin_return_address(0),
           char const* file =__builtin_FILE(),
           int line =__builtin_LINE());
[[noreturn]]
void hardFaultHandler (uint32_t* sp);

void logf (char const* fmt ...);

inline void (*hardFaulter) (uint32_t*) = nullptr;

struct Message {
    uint8_t  mDst =0;
    int8_t   mTag =0;
    uint16_t mLen =0;
    uint8_t* mPtr =nullptr;
    intptr_t mArg =0;
    Message* mLnk =this;

    bool inUse () const { return mLnk != this; }

    Message (const Message&) =delete;
    void operator= (const Message&) =delete;
};
static_assert(sizeof (Message) == 16);

struct Chain {
    bool isEmpty () const { return cHead == nullptr; }
    Message* first () const { return cHead; }

    bool insert (Message& msg);
    bool append (Message& msg);
    bool remove (Message& msg);
    Message* pull ();

protected:
    Message* cHead =nullptr;
};
static_assert(sizeof (Chain) == 4);

struct Task : Message, Chain {
    enum { LIMIT = 30, MARKER = 255 };

    uint8_t tId, owner;      // id of this task and of its owning thread
    Message timer {};        // per-task timer

    Task ();
    // TODO ~Task ();

    bool isThread () const { return tId == owner; }

    virtual void submit (Message& msg);
    virtual int process (Message& msg) =0;

    static Task& byId (uint8_t id);
};
static_assert(sizeof (Task) == 44); // incl 2x Message, Chain, and vtable-ptr

struct Fixer {
    Fixer ();
    ~Fixer ();

    bool saved;
};

struct Lock {
    bool acquire (bool blocking =true);
    void release ();

    bool locked =false;
    Chain waiting;
};

struct Device {
    enum { BASE = '@', LAST = 'Z' };

    uint8_t dId;

    Device (uint8_t id);
    // TODO ~Device ();

    virtual void start (Message&) =0;
    virtual void finish () =0;

    void irqTrigger (uint8_t num);

    static Device& byId (uint8_t id);

protected:
    virtual bool interrupt (int) =0;

    void irqInstall (uint8_t num, uint8_t prio =0x80);
    void reply (Message* mp);
};
static_assert(sizeof (Device) == 8);

namespace sys {
    int svc (int f, int x =0, int y =0, int z =0);

    void send (Message& msg);
    Message& recv ();
    void call (Message& msg);
    void wait (uint16_t ms);

    uint8_t* pool (uint32_t bytes, uint8_t* ptr =nullptr, uint32_t align =4);

    void init (uint32_t* ptr, uint32_t len);
    Message& fork (uint32_t*, uint16_t, int (*)(Message&), intptr_t =0);
    void quit (intptr_t ret =0);

    template< uint32_t N > // see Sys::fork comment
    void init (uint32_t (&stack)[N]) { init(stack, N); }

    // when handed an array as stack, this variant will auto-derive its size
    template< uint32_t N >
    inline static Message& fork (uint32_t (&s)[N], int (*f)(Message&), intptr_t a =0) {
        return fork(s, N, f, a);
    }

} // namespace sys
