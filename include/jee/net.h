namespace jeeh::net {

// uncomment one of these to enable/disable some log output
auto debugf = printf;
//auto debugf = logf;
//auto debugf = [](...) {};

using SmallBuf = char [20];
inline static SmallBuf smallBuf;

struct MacAddr {
    uint8_t b [6];

    auto operator== (MacAddr const& v) const {
        return memcmp(this, &v, sizeof *this) == 0;
    }

    auto asStr (SmallBuf& buf =smallBuf) const {
        auto ptr = buf;
        for (int i = 0; i < 6; ++i)
            ptr += snprintf(ptr, 4, ":%02x", b[i]);
        return buf + 1;
    }
};

struct Net16 {
    constexpr Net16 (uint16_t v =0) : h ((v<<8) | (v>>8)) {}
    operator uint16_t () const { return (h<<8) | (h>>8); }
private:
    uint16_t h;
};

struct Net32 {
    constexpr Net32 (uint32_t v =0) : h1 (v>>16), h2 (v) {}
    operator uint32_t () const { return (h1<<16) | h2; }
private:
    Net16 h1, h2;
};

struct IpAddr : Net32 {
    constexpr IpAddr (uint32_t v =0) : Net32 (v) {}
    constexpr IpAddr (uint8_t v1, uint8_t v2, uint8_t v3, uint8_t v4)
        : Net32 ((v1<<24) | (v2<<16) | (v3<<8) | v4) {}

    auto asStr (SmallBuf& buf =smallBuf) const {
        auto p = (uint8_t const*) this;
        auto ptr = buf;
        for (int i = 0; i < 4; ++i)
            ptr += snprintf(ptr, 5, "%c%d", i == 0 ? ' ' : '.', p[i]);
        return buf + 1;
    }
};

inline static MacAddr const wildMac {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
inline static IpAddr const wildIp {255,255,255,255};

template< typename T >
inline static T& clear (T& obj) {
    memset(&obj, 0, sizeof obj);
    return obj;
}

template <int N>
struct ArpCache {
    void init (IpAddr myIp, MacAddr const& myMac) {
        prefix = myIp & ~0xFF;
        add(myIp, myMac);
    }

    bool canStore (IpAddr ip) const { return (ip & ~0xFF) == prefix; }

    void add (IpAddr ip, MacAddr const& mac) {
        if (canStore(ip)) {
            // may run out of space, start afresh
            if (items[N-1].node != 0)
                memset(items+1, 0, (N-1) * sizeof items[0]);
            uint8_t b = ip;
            for (auto& e : items)
                if (e.node == 0 || b == e.node || mac == e.mac) {
                    e.node = b;
                    e.mac = mac;
                    return;
                }
        }
    }

    MacAddr const& find (IpAddr ip) const {
        if (canStore(ip))
            for (auto& e : items)
                if ((uint8_t) ip == e.node)
                    return e.mac;
        return wildMac;
    }

    struct {
        MacAddr mac;
        uint8_t node; // TODO low byte of IP address, assumes a /24 subnet
    } items [N] {};

    uint32_t prefix =0;
};

using Buffer = uint8_t [1536];
static_assert(sizeof (Buffer) % 32 == 0);

struct Interface {
    MacAddr const mac;
    IpAddr ip, gw, dns, sub;
    uint8_t drv;
    Chain spares;
    Buffer* bufs;
    uint32_t nBufs;
    Message* msgs;
    ArpCache<10> arpCache;

    Interface (MacAddr const& addr, int did, uint32_t size)
            : mac (addr), drv (did) {
        bufs = (Buffer*) sys::pool(size, nullptr, cache::align);
        assert(cache::align == 0 || (uintptr_t) bufs % cache::align == 0);

        nBufs = size / sizeof (Buffer);
        msgs = new Message [nBufs] {};
        debugf(" [intf] nBuf %d bufs %08x bufsz %d msgs %08x\n",
                nBufs, bufs, sizeof (Buffer), msgs);

        for (auto i = 0U; i < nBufs; ++i) {
            auto& m = msgs[i];
            m.mPtr = bufs[i] + 2;
            addToSpares(m);
        }
    }

    bool withinBuffer (void* p) const {
        return bufs <= p && p <= bufs + nBufs;
    }

    Message& asMsg (void* p) const {
        assert(withinBuffer(p));
        // careful: p is NOT aligned to a buffer boundary, rely on truncation
        return msgs[((uintptr_t) p - (uintptr_t) bufs) / sizeof (Buffer)];
    }

    Buffer& asBuffer (void* p) const {
        return bufs[&asMsg(p) - msgs];
    }

    void addToSpares (Message& m) {
        assert(!m.inUse());
        assert(m.mPtr == asBuffer(m.mPtr) + 2);
        m.mLen = sizeof (Buffer) - 2;
        spares.insert(m);
    }

    Interface (Interface const&) =delete; // catch incorrect use
};

struct OptionIter {
    OptionIter (uint8_t* p, uint8_t o =0) : ptr (p), off (o) {}

    operator bool () const { return *ptr != 0xFF; }

    auto next () {
        auto typ = *ptr++;
        len = *ptr++;
        ptr += len;
        return typ;
    }

    void extract (void* p, [[maybe_unused]] uint32_t n =0) {
        assert (n == 0 || n == len);
        memcpy(p, ptr-len, len);
    }

    void append (uint8_t typ, void const* p, uint8_t n) {
        *ptr++ = typ;
        *ptr++ = n + off;
        memcpy(ptr, p, n);
        ptr += n;
        *ptr = off ? 0x00 : 0xFF;
    }

    uint8_t* ptr;
    uint8_t len =0;
    uint8_t off;
};

struct Frame {
    MacAddr dst, src;
    Net16 typ;

    uint8_t* payload () { return (uint8_t*) (this + 1); }

    bool isArp () const { return typ == 0x0806; }
    bool isIp4 () const { return typ == 0x0800; }

    inline static auto& header (Message& m) {
        return *(Frame*) m.mPtr;
    }
};
static_assert(sizeof (Frame) == 14);

struct FrameHandler {
    Interface& ni;
    uint16_t timer =0;
    Message* lingering {}; // completed tx which has a timeout running

    FrameHandler (Interface& intf) : ni (intf) {}

    void tick () {
        if (timer != 0 && --timer == 0) {
            auto p = lingering;
            if (p != nullptr && p->mPtr[-1] > 0) {
                --p->mPtr[-1]; // decrement retries
                sendPkt(*p);    // and resend
            } else {
                p = expired(take(lingering));
                if (p != nullptr)
                    ni.addToSpares(*p);
            }
        }
    }

    bool dispatch (Message& m) {
        if (m.mTag == 'R' && lingering != nullptr) {
            ni.addToSpares(*take(lingering)); // buffer no longer needed
            timer = 0;
        }
        return m.mTag == 'R' ? received(m) : sent(m);
    }

    virtual bool received (Message& m) =0;

    virtual bool sent (Message& m) {
        if (m.mPtr[-2] == 0) // there's no timeout, reuse this buffer
            return false; 

        assert(lingering == nullptr || lingering == &m);
        lingering = &m;
        timer = m.mPtr[-2];
        return true; // don't reuse this lingering packet, yet
    }

    virtual Message* expired (Message* mp) { return mp; }

    Message& prepare (uint8_t* end, uint8_t t =0, uint8_t r =0) {
        auto& m = ni.asMsg(end);
        assert(m.mPtr == ni.asBuffer(end) + 2);

        m.mPtr[-2] = t; // timeout, stored before the actual packet data
        m.mPtr[-1] = r; // ... as well as the number of automatic retries

        m.mDst = ni.drv;
        m.mTag = 'W';
        m.mLen = end - m.mPtr;
        return m;
    }

    void sendPkt (Message& m) {
        sys::send(m);
    }

    void flip (Message& m) {
        auto& h = Frame::header(m);
        h.dst = h.src;
        h.src = ni.mac;
    }

    Frame& init (Message& m, Net16 type) {
        auto& h = Frame::header(m);
        h.dst = wildMac;
        h.src = ni.mac;
        h.typ = type;
        return h;
    }
};

struct Arp {
    Net16 hw, proto;
    uint8_t macLen, ipLen;
    Net16 op;
    MacAddr sendMac;
    IpAddr sendIp;
    MacAddr targMac;
    IpAddr targIp;

    uint8_t* payload () { return (uint8_t*) (this + 1); }

    inline static auto& header (Message& m) {
        return *(Arp*) Frame::header(m).payload();
    }
};
static_assert(sizeof (Arp) == 28);

struct ArpHandler : FrameHandler {
    using FrameHandler::FrameHandler;

    void request (Message& m) {
        auto ip = *(IpAddr*) m.mPtr;
        auto& r = ni.arpCache.find(ip);
        if (&r != &wildMac) { // found in cache
            assert(m.mLen == sizeof (MacAddr));
            *(MacAddr*) m.mPtr = r;
            sys::send(m);
            return;
        }

        pending = &m;
        auto mp = ni.spares.pull();
        assert(mp != nullptr);

        auto& h = init(*mp);
        h.targMac = wildMac;
        h.targIp = ip;
        sendPkt(prepare(h.payload(), 0'2, 2));
    }

    void reply (MacAddr const& mac) {
        assert(pending != nullptr);
        auto& r = *take(pending);
        assert(r.mLen == sizeof (MacAddr));
        *(MacAddr*) r.mPtr = mac;
        sys::send(r);
    }

    bool received (Message& m) override {
        auto& h = Arp::header(m);
        if (h.targIp == ni.ip) {
            SmallBuf sb;
            debugf("ARP %s %s op %d\n",
                    h.sendIp.asStr(), h.sendMac.asStr(sb), +h.op);
            switch (h.op) {
                case 1: // request
                    flip(m);
                    h.op = 2; // reply
                    sendPkt(prepare(h.payload()));
                    return true;
                case 2: { // reply
                    ni.arpCache.add(h.sendIp, h.sendMac);
                    reply(h.sendMac);
                    break;
                }
            }
        }
        return false;
    }

    Message* expired (Message* mp) override {
        reply(wildMac); // failed to locate ip
        return mp;
    }

    void flip (Message& m) {
        FrameHandler::flip(m);
        auto& h = Arp::header(m);
        h.targMac = h.sendMac;
        h.targIp = h.sendIp;
        h.sendMac = ni.mac;
        h.sendIp = ni.ip;
    }

    Arp& init (Message& m) {
        FrameHandler::init(m, 0x0806);
        auto& h = Arp::header(m);
        h.hw = 1;
        h.proto = 0x0800;
        h.macLen = 6;
        h.ipLen = 4;
        h.op = 1;
        h.sendIp = ni.ip;
        h.sendMac = ni.mac;
        return h;
    }

    Message* pending {};
};

struct Ip4 {
    uint8_t versLen, tos;
    Net16 total, id, frag;
    uint8_t ttl, proto;
    Net16 hcheck;
    IpAddr srcIp, dstIp;

    uint8_t* payload () { return (uint8_t*) this + 4 * (versLen & 0x0F); }

    bool isIcmp () const { return proto == 1; }
    bool isUdp  () const { return proto == 17; }
    bool isTcp  () const { return proto == 6; }

    inline static auto& header (Message& m) {
        return *(Ip4*) Frame::header(m).payload();
    }
};
static_assert(sizeof (Ip4) == 20);

struct Ip4Handler : FrameHandler {
    using FrameHandler::FrameHandler;

    void flip (Message& m) {
        FrameHandler::flip(m);
        auto& h = Ip4::header(m);
        h.dstIp = h.srcIp;
        h.srcIp = ni.ip;
    }

    Ip4& init (Message& m, Net16 protocol, IpAddr dIp) {
        FrameHandler::init(m, 0x0800);
        auto& h = clear(Ip4::header(m));
        h.versLen = 0x45;
        h.ttl = 64;
        h.proto = protocol;
        h.srcIp = ni.ip;
        h.dstIp = dIp;

        static uint16_t gid;
        h.id = ++gid;

        return h;
    }

    void sendPkt (Message& m) {
        auto& h = Ip4::header(m);
        h.total = m.mLen - 14;
        FrameHandler::sendPkt(m);
    }
};

struct Icmp {
    uint8_t type, code;
    Net16 sum;

    uint8_t* payload () { return (uint8_t*) (this + 1); }

    inline static auto& header (Message& m) {
        return *(Icmp*) Ip4::header(m).payload();
    }
};
static_assert(sizeof (Icmp) == 4);

struct IcmpHandler : Ip4Handler {
    using Ip4Handler::Ip4Handler;

    bool received (Message& m) override {
        auto& h = Icmp::header(m);
        if (h.type == 8) { // ping
            flip(m);
            h.type = 0; // reply
            h.sum = 0;
            sendPkt(prepare(m.mPtr + m.mLen));
            return true;
        }
        return false;
    }
};

struct Udp {
    Net16 sPort, dPort, len, sum;

    uint8_t* payload () { return (uint8_t*) (this + 1); }

    inline static auto& header (Message& m) {
        return *(Udp*) Ip4::header(m).payload();
    }
};
static_assert(sizeof (Udp) == 8);

struct Tcp {
    Net16 sPort, dPort;
    Net32 seq, ack;
    uint8_t off, code;
    Net16 win, sum, urg;

    uint8_t* payload () { return (uint8_t*) (this + 4 * (code >> 12)); }

    inline static auto& header (Message& m) {
        return *(Tcp*) Ip4::header(m).payload();
    }
};
static_assert(sizeof (Tcp) == 20);

struct PortHandler : Ip4Handler {
    Net16 lPort;
    PortHandler* next {};
    inline static PortHandler* chain;

    PortHandler (Interface& intf, Net16 port =0)
            : Ip4Handler (intf), lPort (port) {
        static uint16_t tport;
        if (port == 0)
            lPort = 0xC000 | tport++; // 49128..65535 temp client port
        listen();
    }

    ~PortHandler () {
        unlisten();
    }

    inline static bool distribute (Message& m) {
        bool read = m.mTag == 'R';
        auto& h1 = Ip4::header(m);
        PortHandler* p = nullptr;
        if (h1.isUdp()) {
            auto& h2 = Udp::header(m);
            p = find(read ? h2.dPort : h2.sPort);
        } else if (h1.isTcp()) {
            auto& h2 = Tcp::header(m);
            p = find(read ? h2.dPort : h2.sPort);
            // TODO also check remote ip, etc
        }
        return p != nullptr ? p->dispatch(m) : false;
    }

    inline static void tickPorts () {
        for (auto p = chain; p != nullptr; p = p->next)
            p->tick();
    }

    // TODO this won't distinguish UDP from TCP ports
    inline static PortHandler* find (Net16 port) {
        for (auto p = chain; p != nullptr; p = p->next)
            if (port == p->lPort)
                return p;
        return nullptr;
    }

    bool received (Message&) override { fail(); return false; }

    void listen () {
        assert(find(lPort) == nullptr);
        next = chain;
        chain = this;
    }

    void unlisten () {
        for (auto p = &chain; *p != nullptr; *p = (*p)->next)
            if (this == *p) {
                *p = next;
                return;
            }
        fail(); // not found
    }
};

struct UdpHandler : PortHandler {
    using PortHandler::PortHandler;

    void flip (Message& m) {
        Ip4Handler::flip(m);
        auto& h = Udp::header(m);
        h.dPort = h.sPort;
        h.sPort = lPort;
    }

    Udp& init (Message& m, IpAddr dIp, Net16 dPort) {
        Ip4Handler::init(m, 17, dIp);
        auto& h = Udp::header(m);
        h.sPort = lPort;
        h.dPort = dPort;
        return h;
    }

    void sendPkt (Message& m) {
        auto& h = Udp::header(m);
        h.len = m.mLen - 34;
        Ip4Handler::sendPkt(m);
    }
};

#include "tcp.h" // TODO keep separate until this has actually been implemented

struct Dhcp {
    uint8_t op, htype, hlen, hops;
    Net32 tid;
    Net16 sec, flags;
    IpAddr clientIp, yourIp, serverIp, gwIp;
    uint8_t clientHw [16], hostName [64], fileName [128];
    Net32 cookie;
    //uint8_t options [308]; // treated as payload

    uint8_t* payload () { return (uint8_t*) (this + 1); }

    inline static auto& header (Message& m) {
        return *(Dhcp*) Udp::header(m).payload();
    }
};
static_assert(sizeof (Dhcp) == 240);

struct DhcpHandler final : UdpHandler {
    enum { Subnet=1,Router=3,Dns=6,Domain=15,Bcast=28,Ntps=42,
            ReqIp=50,Lease=51,MsgType=53,ServerIp=54,ReqList=55 };
    using UdpHandler::UdpHandler;

    void request () {
        auto p = ni.spares.pull();
        assert(p != nullptr);
        auto& h = init(*p);
        memcpy(h.clientHw, &ni.mac, sizeof ni.mac);

        OptionIter it {h.payload()};
        it.append(MsgType, "\x01", 1);         // discover
        it.append(ReqList, "\x01\x03\x06", 3); // subnet router dns

        sendPkt(prepare(it.ptr + 1, 5'0, 3));
    }

    bool received (Message& m) override {
        auto& h = Dhcp::header(m);
        flip(m);
        auto r = h.payload()[2];
        switch (r) {
            case 2: { // offer
                {
                    ni.ip = h.yourIp;
                    OptionIter it {h.payload()};
                    //debugf("dhcp options:");
                    while (it) {
                        auto typ = it.next();
                        //debugf("  %d", typ);
                        switch (typ) {
                            case Subnet: it.extract(&ni.sub, 4); break;
                            case Dns:    it.extract(&ni.dns, 4); break;
                            case Router: it.extract(&ni.gw,  4); break;
                        }
                    }
                }
                ni.arpCache.init(ni.ip, ni.mac);

                h.op = 2; // request
                h.clientIp = h.yourIp;
                memcpy(h.clientHw, &ni.mac, sizeof ni.mac);

                OptionIter it {h.payload()};
                it.append(MsgType, "\x03", 1); // request
                it.append(ServerIp, &h.serverIp, 4);
                it.append(ReqIp, &h.clientIp, 4);

                sendPkt(prepare(it.ptr + 1, 5'0, 3));
                return true;
            }
            case 5: { // ack
                // DHCP server, i.e. gateway
                ni.arpCache.add(Ip4::header(m).srcIp, Frame::header(m).src);

                SmallBuf sb [3];
                debugf("DHCP %s gw %s sub %s dns %s\n",
                        ni.ip.asStr(), ni.gw.asStr(sb[0]),
                        ni.sub.asStr(sb[1]), ni.dns.asStr(sb[2]));
                break;
            }
        }
        return false;
    }

    Message* expired (Message* mp) override {
        if (mp != nullptr)
            timer = 60'0; // try again in 60s
        else
            request(); // repeat forever
        return mp;
    }

    Dhcp& init (Message& m) {
        UdpHandler::init(m, wildIp, 67);
        auto& h = clear(Dhcp::header(m));
        h.op = 1;
        h.htype = 1;
        h.hlen = 6;
        h.flags = 1<<15;
        h.yourIp = wildIp;
        h.cookie = 0x63825363;
        return h;
    }
};

struct Dns {
    Net16 tid, flags, qdc, anc, nsc, arc;
    //uint8_t query [...]; // treated as payload

    uint8_t* payload () { return (uint8_t*) (this + 1); }

    inline static auto& header (Message& m) {
        return *(Dns*) Udp::header(m).payload();
    }
};
static_assert(sizeof (Dns) == 12);

struct DnsHandler final : UdpHandler {
    using UdpHandler::UdpHandler;

    void request (Message& m) {
        pending = &m;
        auto mp = ni.spares.pull();
        assert(mp != nullptr);

        auto& h = init(*mp);
        auto p = h.payload();

        auto mac = ni.arpCache.find(ni.dns);
        Frame::header(*mp).dst = mac;

        auto name = m.mPtr;
        do {
            auto q = ++p;
            while (*name != '.' && *name != 0)
                *p++ = *name++;
            q[-1] = p - q;
        } while (*name++ != 0);
        *p++ = 0;
        *p++ = 0; *p++ = 1;
        *p++ = 0; *p++ = 1;

        h.tid = p - h.payload(); // use query entry length as id
        sendPkt(prepare(p, 1'0, 2));
    }

    void reply (IpAddr ip) {
        assert(pending != nullptr);
        auto& r = *take(pending);
        r.mPtr = (uint8_t*)(uintptr_t) ip; // stash uint32_t reply in ptr field
        sys::send(r);
    }

    bool received (Message& m) override {
        IpAddr ip {};
        auto& h = Dns::header(m);
        if (h.anc > 0 && (h.flags & 0x0F) == 0) { // got answer, no error
            auto p = h.payload() + h.tid;
            if (*p & 0xC0)
                p += 2;
            else
                while (*p++ != 0) {}
            ip = *(IpAddr*) (p+10);
        }
        reply(ip);
        delete this; // lookup complete, delete this DNS listener
        return false;
    }

    Message* expired (Message* mp) override {
        reply(0); // failed to get a response
        return mp;
    }

    Dns& init (Message& m) {
        UdpHandler::init(m, ni.dns, 53);
        auto& h = clear(Dns::header(m));
        h.flags = 0x0100; // query
        h.qdc = 1;
        return h;
    }

    Message* pending {};
};

struct Tftp {
    Net16 op, block;
    // data ...

    uint8_t* payload () { return (uint8_t*) (this + 1); }

    inline static auto& header (Message& m) {
        return *(Tftp*) Udp::header(m).payload();
    }
};
static_assert(sizeof (Tftp) == 4);

struct TftpHandler final : UdpHandler {
    using UdpHandler::UdpHandler;

    void request (Message& m) {
        pending = &m;
        auto mp = ni.spares.pull();
        assert(mp != nullptr);

        auto& h = init(*mp);
        auto p = (char*) &h.block;

        auto mac = ni.arpCache.find(ni.dns);
        Frame::header(*mp).dst = mac;

        strcpy(p, (char const*) m.mArg);
        p += strlen(p) + 1;
        strcpy(p, "octet");
        p += strlen(p) + 1;

        sendPkt(prepare((uint8_t*) p, 3'0, 2));
    }

    void reply (uint32_t len) {
        assert(pending != nullptr);
        auto& r = *take(pending);
        r.mArg = len;
        sys::send(r);
    }

    bool received (Message& m) override {
        auto& h = Tftp::header(m);
        if (h.op == 3) { // got data, no error
            auto n = m.mLen - 46; // UDP + TFTP hdr
            //debugf("tftp #%d, %d b\n", +h.block, n);
            if (h.block == lastBlock + 1) {
                assert(pending != nullptr);
                memcpy(pending->mPtr + 512 * lastBlock++, h.payload(), n);
            }
            flip(m);
            h.op = 4; // ack
            sendPkt(prepare(h.payload()));
            if (n < 512) {
                reply(512 * (lastBlock-1) + n);
                delete this; // transfer complete, delete this TFTP client
            }
            return true;
        }
        debugf("tftp ? %d %d %d b\n", lastBlock, +h.block, +h.op);
        return false;
    }

    Tftp& init (Message& m) {
        UdpHandler::init(m, ni.dns, 9069); // same host as DNS, for now
        auto& h = Tftp::header(m);
        h.op = 1; // rrq
        return h;
    }

    uint16_t lastBlock =0;
    Message* pending {};
};

struct Worker : Task {
    Interface& ni;
    ArpHandler  arph;
    IcmpHandler icmph;
    DhcpHandler dhcph;
    UdpHandler floodh;
    Message* flooder =nullptr;
    Message timer { '@', 'T', };

    Worker (Interface& intf) : ni (intf), arph (ni), icmph (ni),
                                dhcph (ni, 68), floodh (ni) {}

    void process (Message& req) override {
        auto ms = run(req);
        if (ms > 0) {
            assert(!timer.inUse());
            timer.mLen = ms;
            sys::send(timer);
        }
    }

    int run (Message& req) {
        Message mb { ni.drv, 'B', 0, (uint8_t*) &ni.spares };
        sys::send(mb);
        assert(!mb.inUse()); // does not get queued
        if (mb.mLen > 1)
            debugf("used %d spares\n", mb.mLen);

        if (req.mTag == 'I') {
            dhcph.request();
            return 100;
        }

        if (&req == &timer) {
            arph.tick();
            icmph.tick();
            PortHandler::tickPorts(); // UDP and TCP
            return 100;
        }

        if (req.mLen != 60)
            debugf("req %d,%c,%d,%08x,%d\n",
                    req.mDst, req.mTag, req.mLen, req.mPtr, req.inUse());

        if (flooder != nullptr && &req == (Message*) flooder->mPtr) {
            sys::send(*take(flooder)); // flood udp packet can now be reused
            return 0;
        }

        if (req.mDst > Task::LIMIT) {
            assert(req.mDst == ni.drv);
            bool used = false;
            auto& f1 = Frame::header(req);
            if (f1.isArp())
                used = arph.dispatch(req);
            if (f1.isIp4()) {
                auto& f2 = Ip4::header(req);
                if (f2.isIcmp())
                    used = icmph.dispatch(req);
                else
                    used = PortHandler::distribute(req);
            }
            if (!used)
                ni.addToSpares(req);
        } else
            switch (req.mTag) {
                case 'A':
                    arph.request(req);
                    break;
                case 'D': {
                    auto p = new DnsHandler {ni};
                    p->request(req);
                    break;
                }
                case 'F': {
                    assert(req.mPtr != nullptr);
                    auto mp = (Message*) req.mPtr;
                    assert(!mp->inUse());

                    //static IpAddr dst {192,168,178,41};
                    //Frame::header(*mp).dst = { 0xE4,0x5F,0x01,0x14,0xC9,0x62 };
                    static IpAddr dst {192,168,178,6};
                    Frame::header(*mp).dst = { 0xF4,0x4D,0x30,0x6D,0x9B,0x6D };
                    auto& h = floodh.init(*mp, dst, 12345);
                    floodh.sendPkt(floodh.prepare(h.payload() + req.mLen));

                    assert(flooder == nullptr);
                    flooder = &req;
                    //sys::send(req);
                    break;
                }
                case 'L': {
                    auto p = new TcpHandler {ni, 8888};
                    p->request(req);
                    break;
                }
                case 'T': {
                    auto p = new TftpHandler {ni};
                    p->request(req);
                    break;
                }
                default:
                    req.mTag = -1;
                    sys::send(req);
            }

        return 0;
    }
};

} // namespace jeeh::net
