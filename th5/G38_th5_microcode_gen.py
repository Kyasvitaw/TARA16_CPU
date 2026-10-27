#!/usr/bin/env python3
"""
TARA control-ROM generator — CS2310 TH5.

Builds the 2048 x 32-bit control ROM for our microprogrammed TARA CPU.

    ROM address (11 bits) = { Z, N, OPCODE[4:0], TSTATE[3:0] }
                              10  9    8 .. 4      3 .. 0
    Z and N are the LATCHED outputs of status_reg (not the live ALU outputs).

Every control word is built from the named fields below (one field per input pin of
tara_datapath, plus STEP_RESET and HALT for the control unit). Nothing is typed by hand.

Usage
    python3 G<nn>_th5_microcode_gen.py --group 07

Outputs
    G<nn>_th5_control_rom.hex        load into the control ROM (right-click ROM > Load Image)
    G<nn>_th5_microcode_listing.txt  every control word, decoded field by field
    G<nn>_th5_microcode_log.txt      result of the consistency checks
"""
import argparse, os, sys

# ───────────────────────── 1. control-word fields ─────────────────────────
# name: (lowest bit, width).  Names are exactly the input-pin labels of tara_datapath.
FIELDS = {
    # bus sources — at most ONE may be 1 in any control word
    "PC_OUT":     (0, 1),   # PC -> bus
    "IR_OUT":     (1, 1),   # IR -> bus
    "MDR_OUT":    (2, 1),   # MDR -> bus
    "IMM_OUT":    (3, 1),   # imm_unit -> bus
    "RF_OUT":     (4, 1),   # register-file port A (qa) -> bus
    "ALU_OUT":    (5, 1),   # ALU result -> bus
    "RAM_OUT_LD": (6, 1),   # memory output -> bus
    # bus destinations — load on the next rising clock edge
    "MAR_LOAD":   (7, 1),
    "MDR_LOAD":   (8, 1),
    "IR_LOAD":    (9, 1),
    "ACC_load":   (10, 1),
    "RF_WRITE":   (11, 1),  # R[IR[10:8]] <- bus
    # program counter: PC_EN=1,PC_LOAD=0 -> PC+2 ; PC_EN=1,PC_LOAD=1 -> PC<-bus
    "PC_EN":      (12, 1),
    "PC_LOAD":    (13, 1),
    # memory
    "mem_read":   (14, 1),
    "mem_write":  (15, 1),
    # ALU
    "ALU_select": (16, 5),  # ALU opcode (TH2 encoding)
    "ALU_A_SEL":  (21, 1),  # 0 qa, 1 PC
    "ALU_B_SEL":  (22, 1),  # 0 qb, 1 IMM
    "status_we":  (23, 1),  # latch Z N C V into status_reg
    # operand selection
    "IMM_SEL":    (24, 3),  # imm_unit form (see IMM below)
    "RA_SEL":     (27, 2),  # port A index: 0 IR[7:5], 1 IR[10:8], 2 R6, 3 R7
    # control unit only
    "STEP_RESET": (29, 1),  # next edge sends TSTATE back to T0
    "HALT":       (30, 1),  # stop; control word forced to 0 until reset
}
WORD_BITS = 32
BUS_SOURCES = ["PC_OUT", "IR_OUT", "MDR_OUT", "IMM_OUT", "RF_OUT", "ALU_OUT", "RAM_OUT_LD"]
BUS_DESTS = ["MAR_LOAD", "MDR_LOAD", "IR_LOAD", "ACC_load", "RF_WRITE"]

ALU = dict(ADD=9, SUB=10, AND=13, OR=14, XOR=15, NOT=16, SHL=17, SHR=18, SLT=19)
IMM = dict(ZEXT8=0, SEXT8=1, SEXT5=2, REL8X2=3, REL11X2=4, CONST2=5, LIH=6, ZERO=7)
RA = dict(RA=0, RD=1, R6=2, R7=3)

OPCODE = {"NOP": 0, "HLT": 1, "MOV": 2, "LIL": 3, "LIH": 4, "LDW": 5, "STW": 6, "LDB": 7,
          "STB": 8, "ADD": 9, "SUB": 10, "ADDI": 11, "MUL": 12, "AND": 13, "OR": 14,
          "XOR": 15, "NOT": 16, "SHL": 17, "SHR": 18, "SLT": 19, "BZ": 20, "BN": 21,
          "JMP": 22, "CALL": 23, "RET": 24, "PUSH": 25, "POP": 26}
OPNAME = {v: k for k, v in OPCODE.items()}
CORE = ["NOP", "HLT", "MOV", "LIL", "LIH", "ADD", "SUB", "ADDI", "AND", "OR", "XOR",
        "NOT", "SHL", "SHR", "SLT"]


def pack(**signals):
    """pack(PC_OUT=1, MAR_LOAD=1) -> one 32-bit control word."""
    word = 0
    for name, value in signals.items():
        if name not in FIELDS:
            raise KeyError(f"unknown control signal {name!r}")
        lsb, width = FIELDS[name]
        if not 0 <= value < (1 << width):
            raise ValueError(f"{name}={value} does not fit in {width} bits")
        word |= value << lsb
    return word


def unpack(word):
    return {n: (word >> lsb) & ((1 << w) - 1) for n, (lsb, w) in FIELDS.items()}


# ───────────────────────── 2. microprograms ─────────────────────────
SAFE = pack(STEP_RESET=1)                 # filler for every unused address
HALT_WORD = pack(HALT=1, STEP_RESET=1)    # unimplemented-opcode policy

FETCH = [
    pack(PC_OUT=1, MAR_LOAD=1),                              # T0  MAR <- PC
    pack(mem_read=1, RAM_OUT_LD=1, MDR_LOAD=1, PC_EN=1),     # T1  MDR <- M[MAR] ; PC <- PC+2
    pack(MDR_OUT=1, IR_LOAD=1),                              # T2  IR <- MDR
]


def alu_rr(op):
    """Rd <- Ra (op) Rb   (ADD SUB AND OR XOR SLT)"""
    return [pack(RA_SEL=RA["RA"], ALU_A_SEL=0, ALU_B_SEL=0, ALU_select=ALU[op], ALU_OUT=1,
                 RF_WRITE=1, ACC_load=1, status_we=1, STEP_RESET=1)]


def alu_ri(op, imm):
    """Rd <- Rd (op) immediate   (ADDI SHL SHR): port A must read Rd, so RA_SEL = RD"""
    return [pack(RA_SEL=RA["RD"], ALU_A_SEL=0, ALU_B_SEL=1, IMM_SEL=IMM[imm], ALU_select=ALU[op],
                 ALU_OUT=1, RF_WRITE=1, ACC_load=1, status_we=1, STEP_RESET=1)]


# steps from T3 onwards. A step is a word, or a dict {(Z, N): word} for branches (TH6).
EXEC = {
    "NOP":  [SAFE],
    "HLT":  [HALT_WORD],                     # PC already = address + 2 from the fetch
    "MOV":  [pack(RA_SEL=RA["RA"], RF_OUT=1, RF_WRITE=1, STEP_RESET=1)],          # Rd <- Rs
    "LIL":  [pack(IMM_SEL=IMM["ZEXT8"], IMM_OUT=1, RF_WRITE=1, STEP_RESET=1)],    # Rd <- zext(imm8)
    # LIH: port A reads Rd, imm_unit forms {imm8, Rd[7:0]}, written back to Rd (read-modify-write)
    "LIH":  [pack(RA_SEL=RA["RD"], IMM_SEL=IMM["LIH"], IMM_OUT=1, RF_WRITE=1, STEP_RESET=1)],
    "ADD":  alu_rr("ADD"), "SUB": alu_rr("SUB"), "AND": alu_rr("AND"),
    "OR":   alu_rr("OR"),  "XOR": alu_rr("XOR"), "SLT": alu_rr("SLT"),
    "NOT":  [pack(RA_SEL=RA["RA"], ALU_select=ALU["NOT"], ALU_OUT=1, RF_WRITE=1, ACC_load=1,
                  status_we=1, STEP_RESET=1)],                                     # Rd <- ~Rs
    "ADDI": alu_ri("ADD", "SEXT8"),          # immediate is sign-extended
    "SHL":  alu_ri("SHL", "ZEXT8"),          # ALU uses B[4:0] as the shift amount
    "SHR":  alu_ri("SHR", "ZEXT8"),
}


def address(z, n, op, t):
    return (z << 10) | (n << 9) | (op << 4) | t


def build(implemented):
    rom = [SAFE] * 2048
    for z in (0, 1):
        for n in (0, 1):
            for op in range(32):
                for t, word in enumerate(FETCH):
                    rom[address(z, n, op, t)] = word
                name = OPNAME.get(op)
                steps = EXEC[name] if name in implemented else [HALT_WORD]
                for i, step in enumerate(steps):
                    rom[address(z, n, op, 3 + i)] = step[(z, n)] if isinstance(step, dict) else step
    return rom


# ───────────────────────── 3. consistency checks ─────────────────────────
def check(rom, implemented):
    log, failures = [], 0
    U = [unpack(w) for w in rom]

    def report(title, bad):
        nonlocal failures
        failures += bool(bad)
        log.append(f"[{'FAIL' if bad else 'PASS'}] {title}" + (f"  e.g. {bad[:4]}" if bad else ""))

    def steps(op, z, n):            # execute-phase words of one opcode, up to its last state
        out = []
        for t in range(3, 16):
            u = U[address(z, n, op, t)]
            out.append((t, u))
            if u["STEP_RESET"] or u["HALT"]:
                break
        return out

    # 1. at most one bus source
    report("1. no control word selects more than one bus source",
           [hex(a) for a, u in enumerate(U) if sum(u[s] for s in BUS_SOURCES) > 1])

    # 2. every opcode finishes
    bad = []
    for op in range(32):
        for z in (0, 1):
            for n in (0, 1):
                last = steps(op, z, n)[-1][1]
                if not (last["STEP_RESET"] or last["HALT"]):
                    bad.append(f"op{op} Z{z}N{n}")
    report("2. every instruction eventually asserts STEP_RESET or HALT", bad)

    # 3. memory read/write exclusive
    report("3. mem_read and mem_write are never asserted together",
           [hex(a) for a, u in enumerate(U) if u["mem_read"] and u["mem_write"]])

    # 4. write enables only with valid source data
    bad = []
    for a, u in enumerate(U):
        driven = sum(u[s] for s in BUS_SOURCES) == 1
        if any(u[d] for d in BUS_DESTS) and not driven:
            bad.append(f"{hex(a)} load from undriven bus")
        if u["PC_LOAD"] and not (u["PC_EN"] and driven):
            bad.append(f"{hex(a)} PC_LOAD without PC_EN or bus source")
        if u["RAM_OUT_LD"] and not u["mem_read"]:
            bad.append(f"{hex(a)} RAM_OUT_LD without mem_read")
        if u["status_we"] and u["ALU_select"] == 0:
            bad.append(f"{hex(a)} status_we with no ALU operation")
    for op in range(32):                           # MDR must be loaded before it is written to memory
        for z in (0, 1):
            for n in (0, 1):
                loaded = False
                for t, u in steps(op, z, n):
                    if u["mem_write"] and not loaded:
                        bad.append(f"op{op} T{t} mem_write before MDR loaded")
                    loaded |= bool(u["MDR_LOAD"])
    report("4. write enables are never asserted without valid source data", bad)

    # 5. condition independence
    bad = []
    for op in range(32):
        name = OPNAME.get(op)
        for t in range(16):
            w = {(z, n): rom[address(z, n, op, t)] for z in (0, 1) for n in (0, 1)}
            if name == "BZ" and name in implemented:
                ok = w[(0, 0)] == w[(0, 1)] and w[(1, 0)] == w[(1, 1)]
            elif name == "BN" and name in implemented:
                ok = w[(0, 0)] == w[(1, 0)] and w[(0, 1)] == w[(1, 1)]
            else:
                ok = len(set(w.values())) == 1
            if not ok:
                bad.append(f"{name or op} T{t}")
    report("5. conditional-branch words differ only in the relevant condition signal", bad)

    # 6. undefined opcodes are harmless
    state = BUS_DESTS + ["PC_EN", "PC_LOAD", "mem_write", "status_we"]
    bad = [f"op{op} T{t}" for op in range(32) if OPNAME.get(op) not in implemented
           for z in (0, 1) for n in (0, 1) for t, u in steps(op, z, n) if any(u[s] for s in state)]
    report("6. no undefined opcode can modify architectural state (they HALT)", bad)

    # extra: fetch is identical everywhere
    bad = [hex(address(z, n, op, t)) for z in (0, 1) for n in (0, 1) for op in range(32)
           for t in range(3) if rom[address(z, n, op, t)] != FETCH[t]]
    report("7. T0-T2 hold the common fetch sequence for every opcode", bad)
    return log, failures


# ───────────────────────── 4. output files ─────────────────────────
def to_hex(rom):
    """Logisim 'v2.0 raw' image, run-length encoded like Logisim's own files."""
    out, line, i = ["v2.0 raw"], [], 0
    while i < len(rom):
        j = i
        while j < len(rom) and rom[j] == rom[i]:
            j += 1
        line.append(f"{j - i}*{rom[i]:x}" if j - i > 3 else " ".join([f"{rom[i]:x}"] * (j - i)))
        i = j
        if len(line) == 8:
            out.append(" ".join(line)); line = []
    if line:
        out.append(" ".join(line))
    return "\n".join(out) + "\n"


def describe(word):
    u = unpack(word)
    parts = [f"{n}={v}" if FIELDS[n][1] > 1 else n for n, v in u.items() if v]
    return " ".join(parts) or "(nothing)"


def listing(rom, implemented):
    L = ["TARA control ROM (TH5).  address = {Z, N, OPCODE[4:0], TSTATE[3:0]}", "",
         "Control word layout:"]
    L += [f"  bit {lsb:2}{'-' + str(lsb + w - 1) if w > 1 else '':<3}  {n}" for n, (lsb, w) in FIELDS.items()]
    L += ["", f"filler word 0x{SAFE:08X} (STEP_RESET); unimplemented opcode 0x{HALT_WORD:08X} (HALT)", ""]
    for t, w in enumerate(FETCH):
        L.append(f"FETCH T{t}  0x{w:08X}  {describe(w)}")
    for op in range(32):
        name = OPNAME.get(op, f"undefined {op}")
        L.append(f"\n{name}  (opcode {op}, {op:05b})  {'implemented' if name in implemented else 'HALTs'}")
        for t in range(3, 16):
            ws = {(z, n): rom[address(z, n, op, t)] for z in (0, 1) for n in (0, 1)}
            for key, w in (ws.items() if len(set(ws.values())) > 1 else [((None, None), ws[(0, 0)])]):
                tag = f" Z={key[0]} N={key[1]}" if key[0] is not None else ""
                L.append(f"  T{t}{tag}  addr 0x{address(key[0] or 0, key[1] or 0, op, t):03X}  0x{w:08X}  {describe(w)}")
            if all(unpack(w)["STEP_RESET"] or unpack(w)["HALT"] for w in ws.values()):
                break
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--group", default="nn", help="your group number, e.g. 07")
    ap.add_argument("--out", default=".")
    a = ap.parse_args()
    implemented = CORE
    rom = build(implemented)
    log, failures = check(rom, implemented)
    prefix = os.path.join(a.out, f"G{a.group}_th5_")
    text = (f"TARA microcode generator, TH5 — {len(implemented)} instructions: {' '.join(implemented)}\n"
            f"ROM 2048 x {WORD_BITS} bits\n\n" + "\n".join(log) +
            f"\n\n{len(log) - failures}/{len(log)} checks passed\n")
    print(text, end="")
    open(prefix + "microcode_log.txt", "w").write(text)
    if failures:
        sys.exit("checks failed - ROM NOT written")
    open(prefix + "control_rom.hex", "w").write(to_hex(rom))
    open(prefix + "microcode_listing.txt", "w").write(listing(rom, implemented))
    print(f"wrote {prefix}control_rom.hex, microcode_listing.txt, microcode_log.txt")


if __name__ == "__main__":
    main()
