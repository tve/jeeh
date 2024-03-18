Import("env")
#print(env.Dump())

import socket

def rpcSend(sock, cmd):
    sock.sendall((cmd + "\x1A").encode())
    reply = sock.recv(4096)
    assert reply[-1:] == b'\x1A', reply
    return reply[:-1]

def swoDecoder(sock):
    output, remain = "", 0
    while output != "OK":
        msg = sock.recv(4096).decode()
        if msg.startswith("type target_trace data "):
            for c in bytes.fromhex(msg[23:-3]):
                if remain > 0:
                    if chr(c) == "\n":
                        yield output
                        if output in ["OK", "FAIL"]:
                            break
                        output = ""
                    else:
                        output += chr(c)
                    remain -= 1
                else:
                    remain = (c & 0x3) >> 0
                    remain += remain // 3
                    #payload_src = (c & 0x4) >> 2
                    #itm_port = (c & 0xf8) >> 3

def uploader(source, **kwds):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            port = 6666
            s.connect(("127.0.0.1", port))
        except ConnectionRefusedError:
            print(f"pio-extra: could not connect to openocd, port {port}")
            raise
        # source[0] is .bin, source[1] is .elf
        rpcSend(s, f"program {source[1]}; tcl_trace on; reset run")

        for line in swoDecoder(s):
            print(line)

env.AddCustomTarget("upload", "$PROGPATH", uploader)
