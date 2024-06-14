import os, serial, socket, subprocess, sys
Import("env")

#print(env.Dump(), file=sys.stderr)
#print(env.GetProjectOptions(True))

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

def checkViaSwo(source, **kwds):
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
        if env["LDSCRIPT_PATH"][0] == "/":
            print(f"save to FLASH: {env['PIOENV']}", file=sys.stderr)
            t.sendall(f"program {source[0]}\x1A".encode())
        else: # load script does not point to an absolute path
            print(f"load to RAM: {env['PIOENV']}", file=sys.stderr)
            t.sendall(f"reset halt; load_image {source[0]}\x1A".encode())
        while t.recv(1024)[-1:] != b'\x1A':
            pass # consume packets until the final ^Z

        with connectTo(6464) as s:
            t.sendall("resume\x1A".encode())
            b = t.recv(10)
            assert b == b'\x1A', b

            os.makedirs(env["PROJECT_DIR"] + "-out", exist_ok=True)
            out = env.subst("${PROJECT_DIR}-out/${PIOENV}.txt")
            with open(out, "w") as f:
                lines = []
                for line in swoDecoder(s):
                    print(line, file=f)
                    if line == "TEST":
                        isTest = True
                    if len(lines) >= 250:
                        line = "ABORT"
                    lines.append(line)
                    if line in ["OK", "FAIL", "TIMEOUT", "ABORT"]:
                        break

            t.sendall("shutdown\x1A".encode())
            b = t.recv(10)
            assert b == b'\x1A', b

    p.terminate()

    if not isTest or line != "OK":
        for l in lines:
            print(l, file=sys.stderr)
        os.remove(str(source[0])) # force a rebuild next time around
        raise SystemExit(1)

def checkViaUart(source, **kwds):
    isTest = False

    opts = env.GetProjectOptions(True)
    port = opts['monitor_port']
    with serial.Serial(port, 115200, timeout=10) as s:
        s.reset_input_buffer()
        print("pending ...", file=sys.stderr)

        os.makedirs(env["PROJECT_DIR"] + "-out", exist_ok=True)
        out = env.subst("${PROJECT_DIR}-out/${PIOENV}.txt")
        with open(out, "w") as f:
            lines = []
            for line in s:
                try:
                    line = line.decode().strip('\n')
                except UnicodeDecodeError:
                    continue
                print(line, file=f)
                if line == "TEST":
                    isTest = True
                if len(lines) >= 250:
                    line = "ABORT"
                lines.append(line)
                if line in ["OK", "FAIL", "TIMEOUT", "ABORT"]:
                    break

    if not isTest or line != "OK":
        for l in lines:
            print(l, file=sys.stderr)
        os.remove(str(source[0])) # force a rebuild next time around
        raise SystemExit(1)

if 'LOG_UARTC' in env['CPPDEFINES']:
    env.AddCustomTarget("check", "$PROGPATH", checkViaUart, always_build=False)
else:
    env.AddCustomTarget("check", "$PROGPATH", checkViaSwo, always_build=False)
