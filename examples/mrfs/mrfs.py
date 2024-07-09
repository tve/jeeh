#!/usr/bin/env python3
# Minimal Romable File Storage

import io, os, sys, time
from binascii import crc32
from datetime import datetime
from struct import pack, unpack

def addFile(fn):
    info = os.stat(fn)
    size = info.st_size
    t = datetime.fromtimestamp(info.st_mtime)
    date = (((t.year-2000)*100+t.month)*100+t.day)*2048 + t.hour*64 + t.minute

    with open(fn, 'rb') as ifd:
        dat = ifd.read()

    hdr = pack('4s I 16s i I',
               b'MRFS', size, os.path.basename(fn)[:15].encode(), date, 0)
    pad = (31&-size) * b'\xFF'

    crc = crc32(dat)
    crc = crc32(pad, crc)
    crc = crc32(hdr[:28], crc)
    hdr = hdr[:28] + pack('I', crc)

    with open(image, 'rb') as xfd:
        old = xfd.read().rstrip(b'\xFF');
    while len(old) % 32 != 0:
        old += b'\xFF'

    fd.seek(len(old))
    fd.truncate()

    fd.write(hdr)   # 32-byte header
    fd.write(dat)   # payload, i.e. file contents
    fd.write(pad)   # padding to multiple of 32

    fd.write(32*b'\xFF') # append an empty slot

def listing():
    with open(image, 'rb') as fd:
        dat = fd.read()
    files, pos = {}, 0
    while dat[:4] == b'MRFS':
        siz, nam, tim, crc = unpack('I16siI', dat[4:32])
        nam = nam.strip(b'\0').decode()
        end = 32 + siz + (31&-siz)
        dat = dat[end:]
        # only keep last version of each file
        files[nam] = (pos//32, crc, siz,
                      20000000+tim//2048, (tim%2048)//64, tim%64, nam)
        pos += end
    for k in sorted(files):
        v = files[k]
        if v[3] != 0: # skip deleted files
            print('%04X: [%08X] %5db  %8d.%02d%02d  %s' % v)
    print("%04X: %d bytes" % (pos, pos))
    if len(dat) > 0 and dat[:4] != b'\xff\xff\xff\xff':
        raise SystemExit('%s: bad header at offset %d' % (image, pos))

args = sys.argv[1:]
if args:
    image = args.pop(0)
else:
    print("""Usage:
    mrfs.py image           - list contents of image
    mrfs.py image file...   - append file(s) to image""")
    sys.exit(1)

if args:
    for fn in args:
        with open(image, 'ab') as fd:
            addFile(fn)
else:
    listing()
