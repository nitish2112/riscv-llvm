; RUN: llc -mtriple=riscv64 -mcpu=rv64imac -verify-machineinstrs < %s | FileCheck %s

define i32 @clw(i32 *%a) nounwind {
; CHECK-LABEL: clw:
; CHECK: c.lw {{[a-z0-9]+}}, 0(a0)
  %1 = getelementptr i32, i32* %a, i32 0
  %2 = load i32, i32* %1, align 8
  ret i32 %2
}
