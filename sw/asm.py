
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
r_instr_br  = re.compile("\s+(br)\s+(\w+),\s+#0x([A-Fa-f0-9]+)")

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
    return list(filter(lambda l: l.name == s, all_labels))[0]

def cg_replace_labels(xs):
    # assert all items are of type Instr
    assert(all(isinstance(x, Instr) for x in xs))

    i = 0
    x = xs[i]

    while True:
        if x.op == "br":
            print("Replacing br's {0} with index".format(x.ref))
            label = find_str_label(x.ref)
            assert(label)
            assert(label.index >= 0)

            # Build address
            br_mov = Instr()
            br_mov.op = "movi"
            br_mov.rs1 = 5
            br_mov.index = i
            br_mov.imm8 = label.index
            xs.insert(i, br_mov)
            # reformat current br instruction to use rs1 
            x.rs1 = 5
            x.index += 1

            calc_offset(xs)

            i += 1
        try:
            i += 1
            x = xs[i]
        except:
            break


def cg(xs):
    # assert all items are of type Instr
    assert(all(isinstance(x, Instr) for x in xs))

    i = 0
    x = xs[i]

    while True:
        i += 1



        try:
            x = xs[i]
        except:
            break

    return




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

    cg(all_instr)
