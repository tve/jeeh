#!/usr/bin/python3

import sys
from collections import namedtuple

tdMin = 50   # ns
tdMax = 260  # ns
SEC2NSEC = 1_000_000_000

class Timings: pass

Charac = namedtuple('Charac', ['freq','fMin','fMax','dhMin','dvMax','suMin',
                               'lcMin','hcMin','tRise','tFall','dnf'])

characs = [ # freq fMin fMax dhMin dvMax suMin lcMin hcMin tRise tFall dnf
    Charac(  100000,  90000,  110000, 0, 3450, 250, 4700, 4000, 100, 10, 0 ),
    Charac(  400000, 360000,  440000, 0,  900, 100, 1300,  600, 100, 10, 0 ),
    Charac( 1000000, 950000, 1050000, 0,  450,  50,  500,  260,  60,  6, 0 ),
]

valids = []

def compute_presc(tClk, charac):
    tddMin = max(charac.tFall + charac.dhMin - tdMin - ((charac.dnf+3) * tClk), 0)
    tddMax = max(charac.dvMax - charac.tRise - tdMax - ((charac.dnf+4) * tClk), 0)
    tcdMin = charac.tRise + charac.suMin

    psPrev = 16
    for ps in range(16):
        for scld in range(16):
            tcd = (scld + 1) * (ps + 1) * tClk
            if tcd >= tcdMin:
                for sdad in range(16):
                    tdd = (sdad * (ps + 1)) * tClk
                    if tdd >= tddMin and tdd <= tddMax:
                        if ps != psPrev:
                            v = Timings()
                            v.prs = ps
                            v.tcd = scld
                            v.tdd = sdad
                            valids.append(v)
                            psPrev = ps

def compute_scllh (tClk, charac):
    tBus = round(SEC2NSEC / charac.freq)
    cMax = round(SEC2NSEC / charac.fMin)
    cMin = round(SEC2NSEC / charac.fMax)
    dnfd = charac.dnf * tClk
    errPrev = tBus

    ret = None
    for valid in valids:
        tPs = (valid.prs + 1) * tClk
        for scll in range(256):
            tscl_l = tdMin + dnfd + 2 * tClk + (scll + 1) * tPs
            if tscl_l > charac.lcMin and tClk < (tscl_l - tdMin - dnfd) // 4:
                for sclh in range(256):
                    tscl_h = tdMin + dnfd + 2 * tClk + (sclh + 1) * tPs
                    tscl = tscl_l + tscl_h + charac.tRise + charac.tFall
                    if tscl >= cMin and tscl <= cMax and \
                            tscl_h >= charac.hcMin and tClk < tscl_h:
                        err = abs(tscl - tBus)
                        if err < errPrev:
                            errPrev = err
                            valid.scll = scll
                            valid.sclh = sclh
                            ret = valid
    return ret

def getTiming(sysHz, busHz):
    tClk = round(SEC2NSEC / sysHz)
    for charac in characs:
        if busHz >= charac.fMin and busHz <= charac.fMax:
            compute_presc(tClk, charac)
            return compute_scllh(tClk, charac)

def genTimings(mhz):
    r = [f'// {mhz} Mhz:']
    for khzBus in (100, 400, 1000):
        t = getTiming(mhz * 1000000, khzBus * 1000)
        if t:
            #v = (t.prs, t.tcd, t.tdd, t.sclh, t.scll)
            #v = str(t.__dict__).translate({ord(c): None for c in "{':,}"})
            v = ''.join(c for c in str(t.__dict__) if c not in "{':,}")
            x = (t.prs<<28) | (t.tcd<<20) | (t.tdd<<16) | (t.sclh<<8) | t.scll
            r.append(f'case {khzBus:4d}: return 0x{x:08X}; // {v}')
        else:
            r.append(f'// no valid solution for {khzBus} kHz @ {mhz} MHz')
    return r

if __name__ == '__main__':
    for line in genTimings(int(sys.argv[1])):
        print(line)
