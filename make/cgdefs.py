# This module contains project-specific handler definitions for "codegen.py".
# A function named "XYZ" can be used as handler for "//CG xyz ..." lines.
# Return: None = no-op, int = keep N lines, str = 1 line, else list of lines.
# If there is a function called "XYZ_strip", it will be used for stripping.

import os, re, subprocess, sys, xml.dom.minidom

sources = ['../include']

#-------------------------------------------------------- Embedded git version

def VERSION(block):
    v = subprocess.getoutput('git describe --tags --always')
    return 'constexpr auto VERSION = "%s";' % v

def VERSION_strip(block):
    return f'constexpr auto VERSION = "<stripped>";'

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
            if u == "RCC" and re.match(r'A[HP]B\d?L?ENR', rn):
                b = int(byName(x, 'addressOffset'), 0)
                rccs.append((rn, b))
                for f in x.getElementsByTagName('field'):
                    if type(f) is str:
                        continue
                    nn = byName(f, 'name')
                    if nn.endswith('EN'):
                        nn = "EN_" + nn[:-2]
                        if nn == 'EN_DMA':
                            nn += '1' # fix for F302 and L053
                        bb = int(byName(f, 'bitOffset'))
                        enables[nn] = (bb, rn)

    if False:
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
    svdInfo['irqs'] = ["%-22s = %3s," % (t, irqs[t]) \
                            for t in sorted(irqs, key=natsort)] + \
                      [f"limit = {irqLimit}"]
    svdInfo['rccs'] = ["%-8s = 0x%X," % t for t in sorted(rccs)]
    svdInfo['enables'] = ["%-16s = %2d + 8 * %s," % (t, *enables[t]) \
                            for t in sorted(enables)]

def SVD(block, name):
    if 'defines' not in svdInfo:
        parseSvd()
    return svdInfo[name]

#-----------------------------------------------------------------------------
