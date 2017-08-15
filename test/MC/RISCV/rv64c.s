# RUN: llvm-mc %s -triple=riscv64imac -show-encoding \
# RUN:     | FileCheck -check-prefixes=CHECK,CHECK-INST %s
# RUN: llvm-mc -filetype=obj -triple riscv64imac < %s \
# RUN:     | llvm-objdump -d - | FileCheck -check-prefix=CHECK-INST %s

# CHECK-INST: c.ldsp  ra, 8(sp)
# CHECK: encoding: [0xa2,0x60]
c.ldsp  ra, 8(sp)
# CHECK-INST: c.sdsp  ra, 8(sp)
# CHECK: encoding: [0x06,0xe4]
c.sdsp  ra, 8(sp)
# CHECK-INST: c.lw    a3, 8(a2)
# CHECK: encoding: [0x14,0x46]
c.lw    a3, 8(a2)
# CHECK-INST: c.ld    a4, 16(a3)
# CHECK: encoding: [0x98,0x6a]
c.ld    a4, 16(a3)
# CHECK-INST: c.sw    a3, 8(a4)
# CHECK: encoding: [0x14,0xc7]
c.sw    a3, 8(a4)
# CHECK-INST: c.sd    a5, 24(a3)
# CHECK: encoding: [0x9c,0xee]
c.sd    a5, 24(a3)
# CHECK-INST: c.addiw a5, 24
# CHECK: encoding: [0xe1,0x27]
c.addiw a5, 24
# CHECK-INST: c.addw  a1, a2
# CHECK: encoding: [0xb1,0x9d]
c.addw a1, a2
# CHECK-INST: c.subw  a3, a4
# CHECK: encoding: [0x99,0x9e]
c.subw a3, a4
