Import("env")
#print(env.Dump())

import socket

def rpcRecv(sock):
    data = b''
    while data[-1:] != b'\x1A':
        data += sock.recv(1024)
        #print("LEN:", len(data),repr(data[:10]),repr(data[-10:]))
    return data

def swoDecoder(sock):
    output, remain = "", 0
    while output != "OK":
        for msg in rpcRecv(sock).split(b'\x1A'):
            if msg:
                try:
                    _, tag, _, val = msg.split()
                    data = bytes.fromhex(val.decode())
                    #print('GOT:', tag, len(data), repr(data[:30]), repr(data[:10]))
                except:
                    print('OOPS:',len(msg), msg)
                    raise
                if tag == b'target_trace':
                    for c in data:
                        if remain > 0:
                            if chr(c) == "\n":
                                yield output
                                if output == "OK":
                                    break
                                if output == "FAIL":
                                    raise SystemExit(1)
                                output = ""
                            else:
                                output += chr(c)
                            remain -= 1
                        else:
                            remain = c & 0x3
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
        s.sendall(f"program {source[1]}; reset; tcl_trace on\x1A".encode())

        for line in swoDecoder(s):
            print(line)

env.AddCustomTarget("upload", "$PROGPATH", uploader)
