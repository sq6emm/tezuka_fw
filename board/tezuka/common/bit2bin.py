#!/usr/bin/env python3
"""bit2bin.py IN.bit OUT.bin: a Xilinx .bit to the byte-swapped .bin the
Linux zynq-fpga manager takes (header dropped from the first dummy word)."""
import sys
b = open(sys.argv[1], 'rb').read()
i = b.find(b'\xff\xff\xff\xff')
sync = b.find(b'\xaa\x99\x55\x66')
assert 0 <= i < sync, 'no sync word'
d = b[i:]
d += b'\0' * (-len(d) % 4)
out = bytearray(len(d))
out[0::4], out[1::4], out[2::4], out[3::4] = d[3::4], d[2::4], d[1::4], d[0::4]
open(sys.argv[2], 'wb').write(out)
print(len(out), 'bytes')
