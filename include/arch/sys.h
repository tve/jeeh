// Header file for the the central system types and functions.

[[noreturn]]
void fail (char const* f =__builtin_FILE(), int l =__builtin_LINE());

void logf (char const* fmt ...);

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
