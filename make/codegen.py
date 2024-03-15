#!/usr/bin/env python3

# This is a code generator which makes in-place changes to C++ source files.
# The project-specific handlers are in a separate "cgdefs.py" module.

import argparse, os, subprocess, sys, tokenize
from io import StringIO

#sys.path.insert(0, '.')
import cgdefs # found in current directory first, else next to this script

# return a list of tokens to help parse simple text lines
def asTokens(text):
    r = []
    try:
        for t in tokenize.generate_tokens(StringIO(text).readline):
            if t.string:
                r.append(t.string)
    except tokenize.TokenError:
        pass # ignore unbalanced brackets, etc
    return r

# loop over all source lines and generate inline code at each //CG mark
def processLines(lines):
    result = []
    for line in lines:
        if line.strip()[0:4] != '//CG':
            result.append(line)
            continue

        block = []
        head, tag, *params = line.split()
        tail = head[4:]
        if tail in ['', ':']:
            pass
        elif tail == '[':
            while True:
                s = next(lines)
                if s.strip()[0:4] == '//CG':
                    assert s.strip()[4] == ']', 'missing //CG], got: ' + s
                    break
                block.append(s)
        else:
            for _ in range(int(tail)):
                s = next(lines)
                assert s[0:4] != '//CG', 'unexpected //CG, got: ' + s
                block.append(s)

        try:
            func = tag.replace('-','_').upper()
            if cgdefs.stripGen:
                handler = getattr(cgdefs, func + '_strip', lambda *x: [])
            else:
                handler = getattr(cgdefs, func)
        except AttributeError:
            print('unknown tag:', tag)
            return

        n = line.find('//CG')
        prefix = line[:n]

        replacement = handler(block, *params)
        if replacement is None:
            prefix = ''
        elif type(replacement) is int:
            block = block[:replacement] # keep N 1st lines
        else:
            block = replacement
            if type(block) is str:
                block = [block]

        if len(block) > 3:
            head = '//CG['
        elif len(block) > 0:
            head = '//CG%d' % len(block)
        else:
            head = '//CG:'
        out = ' '.join([head, tag, *params])

        result.append(line[:n] + out)
        for s in block:
            result.append(prefix + s)
        if head == '//CG[':
            result.append(line[:n] + '//CG]')
    return result

# process a single file, but only rewrite it if anything has been changed
def processFile(fpath):
    with open(fpath, 'r') as fd:
        lines = [s.rstrip('\n') for s in fd]
    result = list(lines) # make a copy
    if hasattr(cgdefs, 'onFileOpen'):
        cgdefs.onFileOpen(fpath, result)
    result = processLines(iter(result))
    if hasattr(cgdefs, 'onFileDone'):
        cgdefs.onFileDone(fpath, result)
    if result and result != lines:
        print('rewriting:', fpath, file=sys.stderr)
        with open(fpath, 'w') as fd: # FIXME not safe
            for s in result:
                fd.write(s+'\n')

# generate the list of headers to process, expanding and reordering as needed
def listHeaders(args):
    dirs = {}
    for a in args:
        if os.path.isdir(a):
            dirs[a] = os.listdir(a)
            dirs[a].sort()
    files = []
    for a in args:
        if a in dirs or os.path.isfile(a):
            files.append(a)
        else: # it's not an existing file
            for k, v in dirs.items():
                if a in v: # found inside one of the dirs
                    files.append(os.path.join(k, a))
                    v.remove(a)
                    a = None
                    break
            assert not a, "can't find file: " + a
    for a in files:
        if a not in dirs:
            yield a
        else:
            for f in dirs[a]:
                if os.path.splitext(f)[1] == '.h':
                    yield os.path.join(a, f)

# process specified files and options (expanding any directories)
def processAll(d='', e='', o={}, p='', s=False, t=False, v=False, srcs=[]):
    cgdefs.svdFile = d
    cgdefs.projEnv = e
    cgdefs.projOpts = o
    cgdefs.projSrcs = p
    cgdefs.stripGen = s
    cgdefs.testFlag = t
    cgdefs.verbose = v

    if p:
        srcs.insert(0, p)

    if srcs:
        for s in cgdefs.sources:
            if s not in srcs:
                srcs.append(s)
    else:
        srcs = cgdefs.sources

    files = list(listHeaders(srcs)) # convert iterator to list
    if not t and hasattr(cgdefs, 'onStart'):
        cgdefs.onStart(files)

    for f in files:
        if v or t:
            print(f)
        if not t:
            processFile(f)

    if not t and hasattr(cgdefs, 'onFinisn'):
        cgdefs.onFinisn(files)

# main app: parse command line and process specified files and directories
if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument('-d', metavar='FILE', default='',
                    help='System View Description file')
    ap.add_argument('-e', metavar='NAME', default='',
                    help='project options entry')
    ap.add_argument('-f', metavar='FILE', default='',
                    help='project options file')
    ap.add_argument('-p', metavar='DIR', default='',
                    help='project source directory')
    ap.add_argument('-s', action='store_true',
                    help='strip generated code')
    ap.add_argument('-t', action='store_true',
                    help='test run, do not make changes')
    ap.add_argument('-v', action='store_true',
                    help='verbose output')
    ap.add_argument('srcs', nargs='*',
                    help='source files and directories')
    args = vars(ap.parse_args())

    if args['e'] and args['f']:
        import configparser
        config = configparser.ConfigParser()
        config.read_file(open(args['f']))
        args['o'] = dict(config['env:'+args['e']].items())
    del args['f']

    processAll(**args)
