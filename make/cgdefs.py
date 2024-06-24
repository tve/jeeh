# This module contains project-specific handler definitions for "codegen.py".
# A function named "XYZ" can be used as handler for "//CG xyz ..." lines.
# Return: None = no-op, int = keep N lines, str = 1 line, else list of lines.
# If there is a function called "XYZ_strip", it will be used for stripping.

import os, re, subprocess, sys, xml.dom.minidom
from string import Template

sources = ['../include']

#-------------------------------------------------------- Embedded git version

def VERSION(block):
    v = subprocess.getoutput('git describe --tags --always')
    return f'constexpr auto VERSION = "{v}";'

def VERSION_strip(block):
    return 'constexpr auto VERSION = "<stripped>";'

#----------------------------------- Inject board settings from platformio.ini

def BOARD(block, name, suffix=''):
    tag = 'board_' + name
    if tag not in projOpts:
        print('not found:', tag)
        return []
    info = projOpts[tag].split()
    if suffix:
        suffix = "_" + suffix.upper()

    if name == 'leds':
        r = [f'#define LED  "{info[0]}"']
        if len(info) > 1:
            for i, v in enumerate(info):
                r.append(f'#define LED{i+1} "{v}"')
        return r

    if name.startswith('uart'):
        if not suffix:
            suffix = name[4:].upper();
        f = { 'O': '0' }
        for x in info:
            k, v = x.split(':', 1)
            f[k] = v
        # N:USART2 P:A2:7,A3 F:150 V:2 D:1 L:CH O:0 T:1 R:2 C:26,27
        r = [f'#define UART{suffix}_NAME  {f["N"]}',
             f'#define UART{suffix}_PINS  "{f["P"]}"',
             f'#define UART{suffix}_FREQ  {f["F"]}']
        if 'V' in f:
            r.append(f'#define UART{suffix}_VERS  {f["V"]}')
        if 'D' in f:
            t = Template('Irq::DMA${D}_$L$T,Irq::DMA${D}_$L$R,'
                         '$D-1,$T-$O,$R-$O,$C').substitute(f)
            r.append(f'#define UART{suffix}_CONF  ' + t)
        return r

    if name.startswith('i2c'):
        if not suffix:
            suffix = name[3:].upper();
        f = { 'O': '0' }
        f = { 'O': '0' }
        for x in info:
            k, v = x.split(':', 1)
            f[k] = v
        # N:I2C1 P:A2:7,A3 D:1 L:CH O:0 T:1 R:2 C:26,27
        r = [f'#define I2C{suffix}_NAME  {f["N"]}',
             f'#define I2C{suffix}_PINS  "{f["P"]}"',
             f'#define I2C{suffix}_FREQ  {f["F"]}']
        if 'D' in f:
            t = '$N.ADDR,DMA$D.ADDR,$T-$O,$R-$O'
            c = '{ ena::$N,$F,Irq::${N}_EV,Irq::${N}_ER,$D-1,$C }'
            r.append(f'#define I2C{suffix}_TYPE  ' + Template(t).substitute(f))
            r.append(f'#define I2C{suffix}_CONF  ' + Template(c).substitute(f))
        return r

    if name.startswith('spi'):
        if not suffix:
            suffix = name[3:].upper();
        f = { 'O': '0' }
        for x in info:
            k, v = x.split(':', 1)
            f[k] = v
        # N:SPI1 P:A7:5,A6,A5,A4:P F:54 D:1 L:CH O:0 T:4 R:3 C:0,0
        r = [f'#define SPI{suffix}_NAME  {f["N"]}',
             f'#define SPI{suffix}_PINS  "{f["P"]}"',
             f'#define SPI{suffix}_FREQ  {f["F"]}']
        if 'D' in f:
            t = '$N.ADDR,DMA$D.ADDR,$T-$O,$R-$O'
            c = '{ ena::$N,$F,Irq::DMA${D}_$L$T,Irq::DMA${D}_$L$R,$D-1,$C }'
            r.append(f'#define SPI{suffix}_TYPE  ' + Template(t).substitute(f))
            r.append(f'#define SPI{suffix}_CONF  ' + Template(c).substitute(f))
        return r

    # catch-all: "board_foo = bar:123 baz:xyz" will generate:
    #   #define FOO_BAR (123)
    #   #define FOO_BAZ "xyz"
    n = name.upper() + suffix
    r = []
    for x in info:
        if ':' not in x:
            x += ':1'
        k, v = x.split(':', 1)
        k = k.upper()
        if v[0].isdigit():
            r.append(f'#define {n}_{k} ({v})')
        else:
            r.append(f'#define {n}_{k} "{v}"')
    return r

#-------------------------------------------------------------- Parse SVD file

svdInfo = {}

def byName(node, name):
    for x in node.childNodes:
        if x.localName == name:
            return x.firstChild.nodeValue

# see https://stackoverflow.com/questions/4836710
def natsort(s, _nsre=re.compile('([0-9]+)')):
    return [int(text) if text.isdigit() else text.lower()
            for text in _nsre.split(str(s))]

def parseSvd():
    # parse System View Description file
    # see https://www.keil.com/pack/doc/CMSIS/SVD/html/svd_Format_pg.html
    parsed = xml.dom.minidom.parse(svdFile)

    svdName = byName(parsed.getElementsByTagName('device')[0], 'description')
    print('[codegen]', svdName)

    irqs, ioregs, rccs, enables = {}, [], [], {}
    irqLimit = 0

    for p in parsed.getElementsByTagName('peripheral'):
        u = byName(p, 'name').upper()
        b = int(byName(p, 'baseAddress'), 0)
        if u == 'NVIC':
            b = 0xE000E100 # fix: sometimes it's defined as 0xE000E000
        ioregs.append("constexpr IoReg<0x%04X'%04X> %s;" % (b>>16, b&0xFFFF, u))

        interrupts = p.getElementsByTagName('interrupt')
        for x in interrupts:
            n, v = byName(x, 'name'), int(byName(x, 'value'))
            if '_EXTI' in n: # fix U[S]ART<n>_EXTI<m> in F302
                n = n[:n.index('_EXTI')]
            if n.startswith('DMA_STR'): # DMA[1]_STR<n> in H745
                n = n[:3] + '1' + n[3:]
            irqs[n] = v
            if v >= irqLimit:
                irqLimit = v + 1

        registers = p.getElementsByTagName('register')
        for x in registers:
            rn = byName(x, 'name')
            if u == 'RCC' and re.match(r'A[HP]B\d?L?ENR', rn):
                b = int(byName(x, 'addressOffset'), 0)
                rccs.append((rn, b))
                for f in x.getElementsByTagName('field'):
                    if type(f) is str:
                        continue
                    nn = byName(f, 'name').upper();
                    if nn.endswith('EN'):
                        nn = nn[:-2]
                        if nn == 'DMA':
                            nn += '1' # fix for F302 and L053
                        bb = int(byName(f, 'bitOffset'))
                        enables[nn] = (bb, rn)

    hasScb, hasStk = False, False
    for x in ioregs:
        hasScb = hasScb or x.endswith(' SCB;')
        hasStk = hasStk or x.endswith(' STK;')
    if not hasScb:
        ioregs.append("constexpr IoReg<0xE000'ED00> SCB;")
    if not hasStk:
        ioregs.append("constexpr IoReg<0xE000'E010> STK;")

    svdInfo['defines'] = [f'#define STM32   1',
                          f'#define {svdName[:7]} 1',
                          f'#define SVDNAME "{svdName}"']
    svdInfo['ioregs'] = sorted(ioregs, key=lambda s: natsort(s[28:]))
    svdInfo['irqs'] = ['%-22s = %3s,' % (t, irqs[t]) \
                            for t in sorted(irqs, key=natsort)] + \
                      [f'limit = {irqLimit},']
    svdInfo['rccs'] = ['%-8s = 0x%X,' % t for t in sorted(rccs)]
    svdInfo['enables'] = ['%-13s = %2d + 8 * %s,' % (t, *enables[t]) \
                            for t in sorted(enables)]

def SVD(block, name):
    if name not in svdInfo:
        parseSvd()
    return svdInfo[name]

#-----------------------------------------------------------------------------
