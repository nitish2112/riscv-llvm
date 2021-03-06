# RUN: llvm-mc -triple riscv32 < %s -show-encoding \
# RUN:     | FileCheck -check-prefix=INSTR -check-prefix=FIXUP %s
# RUN: llvm-mc -filetype=obj -triple riscv32 < %s \
# RUN:     | llvm-readobj -r | FileCheck -check-prefix=RELOC %s

# Check prefixes:
# RELOC - Check the relocation in the object.
# FIXUP - Check the fixup on the instruction.
# INSTR - Check the instruction is handled properly by the ASMPrinter

.long foo
# RELOC: R_RISCV_32 foo

.quad foo
# RELOC: R_RISCV_64 foo

lui t1, %hi(foo)
# RELOC: R_RISCV_HI20 foo
# INSTR: lui t1, %hi(foo)
# FIXUP: fixup A - offset: 0, value: %hi(foo), kind: fixup_riscv_hi20

addi t1, t1, %lo(foo)
# RELOC: R_RISCV_LO12_I foo
# INSTR: addi t1, t1, %lo(foo)
# FIXUP: fixup A - offset: 0, value: %lo(foo), kind: fixup_riscv_lo12_i

sb t1, %lo(foo)(a2)
# RELOC: R_RISCV_LO12_S foo
# INSTR: sb t1, %lo(foo)(a2)
# FIXUP: fixup A - offset: 0, value: %lo(foo), kind: fixup_riscv_lo12_s

auipc t1, %pcrel_hi(foo)
# RELOC: R_RISCV_PCREL_HI20
# INSTR: auipc t1, %pcrel_hi(foo)
# FIXUP: fixup A - offset: 0, value: %pcrel_hi(foo), kind: fixup_riscv_pcrel_hi20

jal zero, foo
# RELOC: R_RISCV_JAL
# INSTR: jal zero, foo
# FIXUP: fixup A - offset: 0, value: foo, kind: fixup_riscv_jal

bgeu a0, a1, foo
# RELOC: R_RISCV_BRANCH
# INSTR: bgeu a0, a1, foo
# FIXUP: fixup A - offset: 0, value: foo, kind: fixup_riscv_branch
