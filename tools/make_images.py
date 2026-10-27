#!/usr/bin/env python3
"""
Turn a TARA assembly program into the two memory images for G38's ram_word.

    pip install taracpu            (once; it is TARA Studio's own assembler and simulator)
    python3 make_images.py myprog.tara

Writes  myprog_LOW.hex   -> load into the TOP RAM of ram_word    (low byte of each word)
        myprog_HIGH.hex  -> load into the BOTTOM RAM of ram_word (high byte)
and prints the final registers TARA Studio gets, so you can compare with your circuit.
"""
import sys, os

try:
    from src.assembler.asm import assemble
    from src.simulation.cpu import TaraCPU
except ImportError:
    try:
        from taracpu.src.assembler.asm import assemble          # some installs nest the package
        from taracpu.src.simulation.cpu import TaraCPU
    except ImportError:
        sys.exit("TARA Studio not found. Install it first:  pip install taracpu")

if len(sys.argv) != 2:
    sys.exit(__doc__)
src = sys.argv[1]
words, listing, errors, labels = assemble(open(src).read())
if errors:
    for e in errors:
        print(e)
    sys.exit("assembly failed")

img = [0] * 1024
for addr, w in words:
    img[addr // 2] = w
n = max([i for i, w in enumerate(img) if w] + [0]) + 1
base = os.path.splitext(src)[0]
open(base + "_LOW.hex", "w").write("v2.0 raw\n" + "\n".join(f"{w & 0xFF:x}" for w in img[:n]) + "\n")
open(base + "_HIGH.hex", "w").write("v2.0 raw\n" + "\n".join(f"{w >> 8:x}" for w in img[:n]) + "\n")
print(f"wrote {base}_LOW.hex (top RAM) and {base}_HIGH.hex (bottom RAM), {len(words)} instructions")

cpu = TaraCPU()
cpu.load_program(img)
steps = 0
while not cpu.halted and steps < 100000:
    cpu.step(record_history=False)
    steps += 1
print(f"TARA Studio result after {steps} instructions:  PC={cpu.pc:04X}")
print("  " + "  ".join(f"R{i}={cpu.reg[i] & 0xFFFF:04X}" for i in range(8)))
if cpu.error:
    print("  note:", cpu.error)
