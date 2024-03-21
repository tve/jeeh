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

    bool insert (Message& msg);
    bool append (Message& msg);
    bool remove (Message& msg);
    Message* pull ();

protected:
    Message* cHead =nullptr;
};

namespace sys {
    int svc (int f, int x =0, int y =0, int z =0);

    uint8_t* pool (uint32_t bytes, uint8_t* ptr =nullptr, uint32_t align =4);

} // namespace sys
