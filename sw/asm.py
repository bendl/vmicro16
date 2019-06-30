
import sys
import math

import argparse

parser = argparse.ArgumentParser(description='Parse assembly into vmicro16 instruction words.')
parser.add_argument('fname', metavar='fname', type=str, help="Filename containing assembly text")
args = parser.parse_args()
print(args.fname)

# Match lines using regex
import re
r_comment   = re.compile("//.*")
r_label     = re.compile("(\w+):")
r_instr_rr  = re.compile("\s+(\w+)\s+r(\d),\s+r(\d)")
r_instr_ri  = re.compile("\s+(\w+)\s+r(\d),\s+#0x([A-Fa-f0-9]+)")
r_instr_rif = re.compile("\s+(\w+)\s+r(\d),\s+(\w+)")
r_instr_br  = re.compile("\s+(br)\s+(\w+),\s+#0x([A-Fa-f0-9]+)")
r_instr_lw  = re.compile("\s+(\w+)\s+r(\d),\s+r(\d) \+ #0x([A-Fa-f0-9]+)")

all_instr  = []
all_labels = []

class Comment:
    pass

class Label:
    name  = ""
    index = -1

class Instr:
    op      = "NOP"
    rs1     = 0
    rs2     = 0
    imm8    = 0
    imm5    = 0
    index   = -1
    ref     = ""
    def __str__(self):
        return str(self.__class__) + ": " + str(self.__dict__)

def parse_line(l):
    m = r_comment.match(l)
    if m:
        return None

    m = r_instr_lw.match(l)
    if m:
        r = Instr()
        r.op = m.group(1)
        r.rs1 = int(m.group(2))
        r.rs2 = int(m.group(3))
        r.imm8 = int(m.group(4), 16)
        return r

    m = r_label.match(l)
    if m:
        r = Label()
        r.addra = 0
        r.name = m.group(1)
        return r

    m = r_instr_rr.match(l)
    if m:
        r = Instr()
        r.op = m.group(1)
        r.rs1 = int(m.group(2))
        r.rs2 = int(m.group(3))
        return r


    m = r_instr_ri.match(l)
    if m:
        r = Instr()
        r.op = m.group(1)
        r.rs1 = int(m.group(2))
        r.imm8 = int(m.group(3), 16)
        return r

    m = r_instr_br.match(l)
    if m:
        r = Instr()
        r.op = m.group(1)
        r.ref = m.group(2)
        r.imm8 = int(m.group(3), 16)
        return r

    m = r_instr_rif.match(l)
    if m:
        r = Instr()
        r.op = m.group(1)
        r.rs1 = int(m.group(2))
        r.ref = m.group(3)
        return r


def calc_offset(ls):
    lsi = iter(ls)

    index = 0
    for l in lsi:
        if isinstance(l, Instr):
            l.index = index
            index += 1

    lsi = iter(ls)
    for l in lsi:
        if isinstance(l, Label):
            # set label index = next instr index
            n = next(lsi)
            while(not isinstance(n, Instr)):
                n = next(lsi)
            l.index = n.index

def find_str_label(s):
    for l in all_labels:
        if l.name == s:
            return l
    return None

def cg_replace_labels(xs):
    # assert all items are of type Instr
    assert(all(isinstance(x, Instr) for x in xs))

    i = 0
    x = xs[i]

    while True:
        if (x.op == "movi") and x.ref:
            print("Replacing movi's {0} with index".format(x.ref))
            label = find_str_label(x.ref)
            assert(label)
            assert(label.index >= 0)
            x.imm8 = label.index
        try:
            i += 1
            x = xs[i]
        except:
            break


def cg(xs):
    # assert all items are of type Instr
    assert(all(isinstance(x, Instr) for x in xs))

    binstr = []

    for x in xs:
        op = 0
        if x.op == "movi":
            op |= 0b00101 << 11
            op |= x.rs1 << 8
            op |= x.imm8 << 0
            binstr.append(op)
        elif x.op == "mov":
            op |= 0b00100 << 11
            op |= x.rs1 << 8
            op |= x.rs2 << 5
            binstr.append(op)
        elif x.op == "br":
            op |= 0b01000 << 11
            op |= x.rs1 << 8
            op |= x.imm8 << 0
            binstr.append(op)
        elif x.op == "cmp":
            op |= 0b01001 << 11
            op |= x.rs1 << 8
            op |= x.imm8 << 0
            binstr.append(op)
    
    print(binstr)
    return binstr

with open(args.fname, "r") as f:
    # Apply a structure to each line
    lines = list(map(parse_line, f.readlines()))
    print(list(lines))

    # Removes empty information
    lines = list(filter(lambda x: x != None, lines))

    # Calculates instruction offsets
    calc_offset(list(lines))

    all_instr  = list(filter(lambda x: isinstance(x, Instr), lines))
    all_labels = list(filter(lambda x: isinstance(x, Label), lines))

    print(list(lines))
    print(list(all_instr))
    print(list(all_labels))

    for i in all_instr:
        print(i)

    cg_replace_labels(all_instr)
    for i in all_instr:
        print(i)

    binstr = cg(all_instr)
    for b in binstr:
        print("{:04x}\n".format(b))

