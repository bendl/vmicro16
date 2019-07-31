
import argparse
import serial


parser = argparse.ArgumentParser(
    description='Parse assembly into vmicro16 instruction words.')
parser.add_argument('port', 
    metavar='port', 
    type=str, 
    help="COM/tty port. E.g. COM8")

parser.add_argument('hexf', 
    metavar='hexf', 
    type=str, 
    help="Filename for hex file")

args = parser.parse_args()
print(args.port)

with serial.Serial('COM8', 115200, timeout=1) as ser:
    print("connected!")

    with open(args.hexf, "r") as f:
        lines = list(map(lambda l: l.strip(), f.readlines()))
        lines = list(map(lambda l: int(l, 16), lines))

        # Padding
        rem   = 64 - len(lines)
        print(len(lines), rem)
        assert(rem >= 0)

        for l in lines:
            assert(l <= 0xffff)

            low  = l & 0xff;
            high = (l & 0xff00) >> 8;
            print(hex(high), hex(low))

            # send low byte first
            ser.write(low)
            ser.write(high)

        # Fill up remaining blanks
        for i in range(rem):
            ser.write(0)
            ser.write(0)

        print("Done!")

