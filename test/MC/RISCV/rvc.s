# RUN: llvm-mc %s -triple=riscv32imac -show-encoding \
# RUN:     | FileCheck -check-prefixes=CHECK,CHECK-INST %s
# RUN: llvm-mc -filetype=obj -triple riscv32imac < %s \
# RUN:     | llvm-objdump -d - | FileCheck -check-prefix=CHECK-INST %s

# CHECK-INST: c.lwsp  ra, 12(sp)
# CHECK: encoding: [0xb2,0x40]
c.lwsp  ra, 12(sp)
