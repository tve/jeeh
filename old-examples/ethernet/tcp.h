struct TcpHandler : PortHandler {
    using PortHandler::PortHandler;

    void request (Message& m) {
        debugf("  tcp req\n");
        (void) m;
    }

    bool received (Message& m) override {
        logDump(m.mPtr, m.mLen, "tcp RECV");
        return false;
    }
};
