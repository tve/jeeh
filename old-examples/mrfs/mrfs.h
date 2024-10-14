// A "Mapped Romable File Store" for embedded flash memory.
#pragma once

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>

template< typename FS >
struct Mrfs {
    struct File {
        enum { MAGIC = 0x5346524D }; // 'MRFS'

        File () { memset(this, 0, sizeof *this); }

        uint32_t magic, size;
        char name [15], zero;
        uint32_t time, check;

        uint8_t const* begin () const { return (uint8_t const*) (this+1); }
        uint8_t const* end () const { return begin() + size; }
    };
    static_assert(sizeof (File) == 32);

    FS& fs;
    uint32_t fill; // next unused position

    File scan, in, out; // out.check != 0 when in progress
    uint8_t buf [32]; // collect bytes to write in 32-byte chunks

    int unused () { return fs.size - fill - 2*32; }

    void load (uint32_t pos, File& fd) {
        assert(pos < fs.size);
        fs.read(pos, &fd, sizeof fd);
        if (fd.magic != File::MAGIC)
            pos = ~0; // bogus position
        fd.magic = pos;
    }

    uint32_t next (uint32_t pos, File& fd) {
        load(pos, fd);
        return fd.magic == pos ? pos + 32 + fd.size + (0x1F & -fd.size) : ~0;
    }

    // TODO sole dependency on jeeh
    inline static uint32_t time10d () {
        auto dt = jeeh::rtc::getDate();
        auto d = 10000*dt.yr + 100*dt.mo + dt.dy;
        auto t = 64*dt.hh + dt.mm;
        return (d<<11) + t;
    }

    Mrfs (FS& f) : fs (f), fill (0) {
        while (true) {
            auto x = next(fill, scan);
            if (x == 0 || x >= fs.size)
                break;
            fill = x;
        }
    }

    File const* open (char const* name =nullptr) {
        // an open output file always overrides any others
        if (name == nullptr)
            return out.magic != 0 ? &out : nullptr;

        // go over all entries, looking for the last filename match
        uint32_t p = fs.size;
        for (auto pos = 0U; pos < fill; pos = next(pos, in)) {
            load(pos, in);
            if (strcmp(name, in.name) == 0)
                p = in.time != ~0U ? pos : fs.size; // might be deleted
        }
        if (p == fs.size)
            return nullptr;
        in.magic = p;
        return &in;
    }

    int create (char const* name =nullptr) {
        if (name != nullptr) {
            if (out.magic != 0)
                return -1;
            out.magic = File::MAGIC;
            strncpy(out.name, name, sizeof out.name);
        }
        return unused();
    }

    int write (void const* ptr, uint32_t len) {
        if (out.magic != File::MAGIC || len > unused() - out.size)
            return -1;
        for (auto i = 0U; i < len; ++i) {
            auto pos = out.size % sizeof buf;
            if (pos == 0)
                memset(buf, 0xFF, sizeof buf);
            buf[pos] = ((uint8_t const*) ptr)[i];
            ++out.size;
            if (++pos == sizeof buf)
                fs.write(fill + out.size, buf, sizeof buf);
        }
        return len;
    }

    int close (uint32_t time =0) {
        if (out.magic != File::MAGIC)
            return -1;
        auto sz = out.size;
        while (out.size % sizeof buf != 0)
            write("\xFF", 1); // make sure last data gets flushed
        out.size = sz;
        if (out.time == 0)
            out.time = time != 0 ? time : time10d();
        fs.write(fill, &out, sizeof out);
        fill = next(fill, out);
        memset(&out, 0, sizeof out);
        return 0;
    }

    int remove (char const* name =nullptr) {
        auto p = open(name);
        if (p == nullptr)
            return -1;
        if (p != &out)
            create(name);
        out.time = ~0;
        return close();
    }

    File* readDir (File* curr =nullptr) {
        assert(curr == nullptr || curr == &scan);
        auto pos = curr == nullptr ? 0 : next(scan.magic, scan);
        load(pos, scan);
        while (pos < fill && open(scan.name)->magic != pos)
            pos = next(pos, scan); // skip obsolete entries
        load(pos, scan);
        return pos < fill ? &scan : nullptr;
    }
};
