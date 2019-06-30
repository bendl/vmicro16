
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
        if x.ref:
            # it might be a label
            label = find_str_label(x.ref)
            if label:
                assert(label.index >= 0)
                x.imm8 = label.index
            else:
                label = cg_str_to_imm(x.ref)
                if label != None:
                    x.imm8 = label
                else:
                    sys.stderr.write("Unknown label '{:s}'".format(x.ref))
        try:
            i += 1
            x = xs[i]
        except:
            break

def cg_str_to_imm(str):
    if str == "BR_U":
        return 0
    elif str == "BR_E":
        return 1
    elif str == "BR_NE":
        return 2
    elif str == "BR_G":
        return 3
    elif str == "BR_GE":
        return 4
    elif str == "BR_L":
        return 5
    elif str == "BR_LE":
        return 6
    elif str == "BR_S":
        return 7
    elif str == "BR_S":
        return 8
    else:
        sys.stderr.write("cg_str_to_imm for {:s} not implemented!".format(str))
        return None

def cg(xs):
    # assert all items are of type Instr
    assert(all(isinstance(x, Instr) for x in xs))

    binstr = []

    for x in xs:
        print("Cg for {:s}".format(x.op))
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
        elif x.op == "lw":
            op |= 0b00001 << 11
            op |= x.rs1 << 8
            op |= x.rs2 << 5
            assert(x.imm8 >= -16 and x.imm8 <= 15)
            op |= x.imm8 << 0
            binstr.append(op)
        elif x.op == "sw":
            op |= 0b00010 << 11
            op |= x.rs1 << 8
            op |= x.rs2 << 5
            assert(x.imm8 >= -16 and x.imm8 <= 15)
            op |= x.imm8 << 0
            binstr.append(op)
        elif x.op == "halt":
            op |= 0b01100 << 11
            op |= x.rs1 << 8
            op |= x.rs2 << 5
            op |= x.imm8 << 0
            binstr.append(op)
        elif x.op == "lwex":
            op |= 0b01101 << 11
            op |= x.rs1 << 8
            op |= x.rs2 << 5
            assert(x.imm8 >= -16 and x.imm8 <= 15)
            op |= x.imm8 << 0
            binstr.append(op)
        elif x.op == "swex":
            op |= 0b01101 << 11
            op |= x.rs1 << 8
            op |= x.rs2 << 5
            assert(x.imm8 >= -16 and x.imm8 <= 15)
            op |= x.imm8 << 0
            binstr.append(op)
        else:
            sys.stderr.write("Cg for '{:s}' not implemented!".format(x.op))

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

    print("Found {:d} LABELS".format(len(all_labels)))
    print("Found {:d} INSTR".format(len(all_instr)))

    for i in all_instr:
        print(i)

    print("Replacing labels...")
    cg_replace_labels(all_instr)
    for i in all_instr:
        print(i)

    # Write hex words to verilog memh file
    binstr = cg(all_instr)
    with open("asm.s.hex", "w") as out:
        for i, b in enumerate(binstr):
            print("{:d}\t{:04x}".format(i, b))
            out.write("{:04x}\n".format(b))
        print("Written asm.s.hex file!")


