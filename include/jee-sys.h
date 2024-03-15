// Header file for the the central system types and functions.

[[noreturn]]
void fail (char const* f =__builtin_FILE(), int l =__builtin_LINE());

int debugf (char const* fmt ...);

struct Message {
    int8_t    mDst =0;
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

    bool insert (Message& msg) {
        assert(!msg.inUse());
        msg.mLnk = cHead;
        cHead = &msg;
        return msg.mLnk == nullptr;
    }

    bool append (Message& msg) {
        assert(!msg.inUse());
        auto pp = &cHead;
        while (*pp != nullptr)
            pp = &(*pp)->mLnk;
        msg.mLnk = nullptr;
        *pp = &msg;
        return pp != &cHead;
    }

    bool remove (Message& msg) {
        assert(msg.inUse());
        for (auto pp = &cHead; *pp != nullptr; pp = &(*pp)->mLnk)
            if (*pp == &msg) {
                *pp = msg.mLnk;
                msg.mLnk = &msg;
                return true;
            }
        return false;
    }

    Message* pull () {
        auto mp = cHead;
        if (mp != nullptr) {
            cHead = mp->mLnk;
            mp->mLnk = mp;
        }
        return mp;
    }

protected:
    Message* cHead =nullptr;
};

namespace sys {

    void send (Message& msg);
    Message& recv ();
    void call (Message& msg);
    void wait (uint16_t ms);

    int currId ();

} // namespace sys

struct Task : Message, Chain {
    enum { BASE = 64, MAX_TASKS = 50, MARKER = -128 };

    int8_t id;
    uint8_t owner;
    Message timer { -1, 'T' };

    Task () : Message { MARKER } {
        for (auto i = 0; i < MAX_TASKS; ++i)
            if (tasks[i] == nullptr) {
                tasks[i] = this;
                id = BASE + i;
                return;
            }
        fail(); // too many tasks
    }

    virtual ~Task () {
        tasks[id-BASE] = nullptr;
    }

    static Task& byId (int i) {
        assert(BASE <= i && i < BASE + MAX_TASKS);
        assert(tasks[i-BASE] != nullptr);
        return *tasks[i-BASE];
    }

    void init () {
        owner = sys::currId();
        Message m { id, 'I' };
        sys::send(m);
    }

    void send (Message& m) {
        auto prev = active;
        active = id;
        auto ms = process(m);
        if (ms > 0) {
            timer.mLen = ms;
            sys::send(timer);
        }
        active = prev;
    }

    virtual int process (Message&) =0;

    inline static int8_t active;
    inline static Task* tasks [MAX_TASKS];
};
