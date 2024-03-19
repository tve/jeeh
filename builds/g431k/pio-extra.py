Import("env")
#print(env.Dump())

import socket

def swoDecoder(sock):
    sock.settimeout(3)
    output, remain = "", 0
    while output != "OK":
        data = sock.recv(8192)
        if not data:
            break
        #if data[:2] == b'\x00\x00':
        #    print('pio-extra: GOT', len(data), data[:5],data[-5:])
        if data[-2:] == b'\x00\x00':
            continue
        for c in data:
            if remain > 0:
                if chr(c) == "\n":
                    yield output
                    if output == "OK":
                        break
                    if output == "FAIL":
                        raise SystemExit(1)
                    output = ""
                elif c:
                    output += chr(c)
                remain -= 1
            else:
                remain = c & 0x3
                remain += remain // 3
                #payload_src = (c & 0x4) >> 2
                #itm_port = (c & 0xf8) >> 3

def uploader(source, **kwds):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as t:
        try:
            port = 6666
            t.connect(("127.0.0.1", port))
        except ConnectionRefusedError:
            print(f"pio-extra: could not connect to openocd, port {port}")
            raise

        # source[0] is .bin, source[1] is .elf
        t.sendall(f"program {source[1]}\x1A".encode())
        b = t.recv(10)
        assert b == b'\x1A', b

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                port = 6464
                s.connect(("127.0.0.1", port))
            except ConnectionRefusedError:
                print(f"pio-extra: could not connect to openocd, port {port}")
                raise

            # source[0] is .bin, source[1] is .elf
            t.sendall(f"resume\x1A".encode())
            b = t.recv(10)
            assert b == b'\x1A', b

            for line in swoDecoder(s):
                print(line)

env.AddCustomTarget("upload", "$PROGPATH", uploader)
