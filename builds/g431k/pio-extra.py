Import("env")
#print(env.Dump())

import socket, subprocess, sys

def connectTo(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect(("127.0.0.1", port))
    except ConnectionRefusedError:
        print(f"pio-extra: could not connect to openocd, port {port}",
                file=sys.stderr)
        raise
    return sock

def flushInput(sock):
    sock.settimeout(0)
    try:
        while sock.recv(1024):
            pass
    except BlockingIOError:
        pass
    sock.settimeout(None)

def swoDecoder(sock):
    sock.settimeout(0.5)
    output, remain = "", 0
    while True:
        try:
            data = sock.recv(8192)
        except TimeoutError:
            yield "TIMEOUT"
            break
        if data[:2] == b'\x00\x00':
            print('pio-extra: GOT', len(data), data[:5],data[-5:],
                  file=sys.stderr)
        if data[-2:] == b'\x00\x00':
            continue
        for c in data:
            if remain > 0:
                if chr(c) == "\n":
                    yield output
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
    # TODO yuck: launch openocd for EACH run because of null-bytes issue :(
    p = subprocess.Popen(env['PROJECT_PACKAGES_DIR'] +
                         "/tool-openocd/bin/openocd",
                         stderr=subprocess.PIPE)

    while True:
        s = p.stderr.readline()
        #print(s.decode(), end='')
        if not s or b' port 6464 ' in s:
            break

    isTest = False

    with connectTo(6666) as t:
        # source[0] is .bin, source[1] is .elf
        if env["LDSCRIPT_PATH"][0] == "/":
            print(f"{env['PIOENV']}: save to FLASH", file=sys.stderr)
            t.sendall(f"program {source[1]}\x1A".encode())
        else: # load script does not point to an absolute path
            print(f"{env['PIOENV']}: load to RAM", file=sys.stderr)
            t.sendall(f"reset halt; load_image {source[1]}\x1A".encode())
        b = t.recv(200)
        assert b[-1:] == b'\x1A', b

        with connectTo(6464) as s:
            # source[0] is .bin, source[1] is .elf
            t.sendall("resume\x1A".encode())
            b = t.recv(10)
            assert b == b'\x1A', b

            for line in swoDecoder(s):
                if line == "TEST":
                    isTest = True
                if isTest:
                    print(line, flush=True)
                else:
                    print(line, file=sys.stderr)
                if line in ["OK", "FAIL", "TIMEOUT"]:
                    break

            t.sendall("shutdown\x1A".encode())
            b = t.recv(10)
            assert b == b'\x1A', b

    p.terminate()

    if not isTest or line in ["FAIL", "TIMEOUT"]:
        raise SystemExit(1)

env.AddCustomTarget("upload", "$PROGPATH", uploader)
