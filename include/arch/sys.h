// Header file for the the central system types and functions.

[[noreturn]]
void fail (char const* f =__builtin_FILE(), int l =__builtin_LINE());

void logf (char const* fmt ...);
void hardFaultHandler (uint32_t* sp);

struct Message {
    uint8_t   mDst =0;
    int8_t    mTag =0;
    uint16_t  mLen =0;
    uint8_t*  mPtr =nullptr;
    uintptr_t mArg =0;
    Message*  mLnk =this;

    bool inUse () const { return mLnk != this; }

    Message (const Message&) =delete;
    void operator= (const Message&) =delete;
};

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

struct Device {
    enum { BASE = '@', LAST = 'Z' };

    uint8_t dId;

    Device (uint8_t id);
    // TODO ~Device ();

    static constexpr auto asIndex (uint8_t id) {
        assert(BASE <= id && id <= LAST);
        return id - BASE;
    }

    virtual void start (Message&) =0;
    virtual void finish () =0;
    virtual bool interrupt (int) =0;

    void irqInstall (uint8_t num, uint8_t prio =0x80);
    void irqTrigger (uint8_t num);
    void reply (Message* mp);

    static Device& byId (uint8_t id);
    static void process ();

    static uint32_t pending;
    static Device* devices [];
    static uint8_t interrupts [];
};

struct Task : Message, Chain {
    enum { LIMIT = 20 };

    uint8_t tid;

    Task ();
    // TODO ~Task ();

    static Task& byId (uint8_t id);
};

namespace sys {
    int svc (int f, int x =0, int y =0, int z =0);

    uint8_t* pool (uint32_t bytes, uint8_t* ptr =nullptr, uint32_t align =4);
    void send (Message& msg);
    Message& recv ();
    void call (Message& msg);
    void wait (uint16_t ms);

} // namespace sys
