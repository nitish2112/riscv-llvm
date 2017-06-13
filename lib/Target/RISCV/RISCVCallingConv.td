//===-- RISCVCallingConv.td - Calling Conventions RISCV ----*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This describes the calling conventions for the RISCV architecture.
//
//===----------------------------------------------------------------------===//

/// CCIfAlign - Match of the original alignment of the arg
class CCIfAlign<string Align, CCAction A>:
  CCIf<!strconcat("ArgFlags.getOrigAlign() == ", Align), A>;

class CCIfNotAlign<string Align, CCAction A>:
  CCIf<!strconcat("ArgFlags.getOrigAlign() != ", Align), A>;

// RISCV 32-bit C return-value convention.
def RetCC_RISCV32 : CallingConv<[
  CCIfType<[i1, i8, i16], CCPromoteToType<i32>>,
  CCIfType<[v1i64, v2i32, v4i16, v8i8, v2f32], CCBitConvertToType<f64>>,
  CCIfType<[i32], CCAssignToReg<[X10_32, X11_32]>>,
  CCIfType<[i64], CCAssignToRegWithShadow<[X10_32], [X11_32]>>,
  CCIfType<[f32], CCBitConvertToType<i32>>
]>;

// RISCV 32-bit C Calling convention for variable argument.
def CC_RISCV32_VAR : CallingConv<[
  CCIfType<[i1, i8, i16], CCPromoteToType<i32>>,

  // Pass by value if the byval attribute is given
  CCIfByVal<CCPassByVal<4, 4>>,

  // i64/f64 is passed in even pairs of GPRs
  // i64 is 8-aligned i32 here, so we may need to eat R1 as a pad register
  // (and the same is true for f64 if VFP is not enabled)
  CCIfType<[i32], CCIfNotAlign<"8",
                     CCAssignToReg<[X10_32, X11_32, X12_32, X13_32,
                                    X14_32, X15_32, X16_32, X17_32]>>>,

  CCIfType<[i32], CCIfAlign<"8",
           CCAssignToRegWithShadow<[X10_32, X12_32, X14_32, X16_32],
                                   [X10_32, X11_32, X13_32, X15_32]>>>,

  // If a part of argument assign to stack, shadow the argument registers.
  // To avoid a double argument assign to stack and register in the same time.
  CCIfType<[i32], CCIfAlign<"8", CCAssignToStackWithShadow<4, 8,
                                         [X10_32, X11_32, X12_32, X13_32,
                                          X14_32, X15_32, X16_32, X17_32]>>>,

  CCIfType<[f32], CCBitConvertToType<i32>>,
  CCIfType<[i64, f64], CCAssignToRegWithShadow<[X10_32, X12_32, X14_32, X16_32],
                                               [X11_32, X13_32, X15_32, X17_32]>>,

  CCIfType<[i32, f32], CCAssignToStack<4, 4>>,
  CCIfType<[i64, f64], CCAssignToStack<8, 8>>
]>;

def CC_RISCV32 : CallingConv<[
  CCIfType<[i1, i8, i16], CCPromoteToType<i32>>,

  // Pass by value if the byval attribute is given
  CCIfByVal<CCPassByVal<4, 4>>,

  CCIfType<[i32], CCAssignToReg<[X10_32, X11_32, X12_32, X13_32,
                                 X14_32, X15_32, X16_32, X17_32]>>,

  // If a part of argument assign to stack, shadow the argument registers.
  // To avoid a double argument assign to stack and register in the same time.
  CCIfType<[i32], CCIfAlign<"8", CCAssignToStackWithShadow<4, 8,
                                         [X10_32, X11_32, X12_32, X13_32,
                                          X14_32, X15_32, X16_32, X17_32]>>>,

  CCIfType<[f32], CCBitConvertToType<i32>>,
  CCIfType<[i64, f64], CCAssignToRegWithShadow<[X10_32, X12_32, X14_32, X16_32],
                                               [X11_32, X13_32, X15_32, X17_32]>>,

  CCIfType<[i32, f32], CCAssignToStack<4, 4>>,
  CCIfType<[i64, f64], CCAssignToStack<8, 8>>
]>;

// RISCV 64-bit C return-value convention.
def RetCC_RISCV64 : CallingConv<[
  CCIfType<[i8, i16, i32, i64], CCIfInReg<CCPromoteToType<i64>>>,
  CCIfType<[i64], CCAssignToReg<[X10_64, X11_64]>>,
  CCIfType<[i128], CCAssignToRegWithShadow<[X10_64], [X11_64]>>,

  CCIfType<[v1i64, v2i32, v4i16, v8i8, v2f32], CCBitConvertToType<f64>>,
  CCIfType<[f32], CCPromoteToType<f64>>,
  CCIfType<[f64], CCAssignToReg<[X10_64, X11_64]>>,
  CCDelegateTo<RetCC_RISCV32>
]>;

// RISCV 64-bit C Calling convention for variable argument.
def CC_RISCV64_VAR : CallingConv<[
  CCIfType<[f32], CCBitConvertToType<i32>>,
  CCIfType<[i1, i8, i16, i32], CCPromoteToType<i64>>,

  // Pass by value if the byval attribute is given
  CCIfByVal<CCPassByVal<8, 8>>,

  // i128 is split to two i64s
  CCIfType<[i64], CCIfSplit<CCAssignToRegWithShadow<[X10_64, X12_64, X14_64, X16_64],
                                                    [X10_64, X11_64, X13_64, X15_64]>>>,

  // i128 is split to two i64s, and its stack alignment is 16 bytes.
  CCIfType<[i64], CCIfSplit<CCAssignToStackWithShadow<8, 16, [X17_64]>>>,

  CCIfType<[i64], CCAssignToReg<[X10_64, X11_64, X12_64, X13_64,
                                 X14_64, X15_64, X16_64, X17_64]>>,

  // If a part of argument assign to stack, shadow the argument registers.
  // To avoid a double argument assign to stack and register in the same time.
  CCIfType<[i64], CCIfAlign<"16", CCAssignToStackWithShadow<8, 16,
                                         [X10_64, X11_64, X12_64, X13_64,
                                          X14_64, X15_64, X16_64, X17_64]>>>,

  CCIfType<[i64, f64], CCAssignToRegWithShadow<[X10_64, X12_64, X14_64, X16_64],
                                               [X11_64, X13_64, X15_64, X17_64]>>,

  CCIfType<[i64, f64], CCAssignToStack<8, 8>>
]>;

def CC_RISCV64 : CallingConv<[
  CCIfType<[f32], CCBitConvertToType<i32>>,
  CCIfType<[i1, i8, i16, i32], CCPromoteToType<i64>>,

  // Pass by value if the byval attribute is given
  CCIfByVal<CCPassByVal<8, 8>>,

  CCIfType<[i64], CCAssignToReg<[X10_64, X11_64, X12_64, X13_64,
                                 X14_64, X15_64, X16_64, X17_64]>>,

  // If a part of argument assign to stack, shadow the argument registers.
  // To avoid a double argument assign to stack and register in the same time.
  CCIfType<[i64], CCIfAlign<"16", CCAssignToStackWithShadow<8, 16,
                                         [X10_64, X11_64, X12_64, X13_64,
                                          X14_64, X15_64, X16_64, X17_64]>>>,

  CCIfType<[i64, f64], CCAssignToRegWithShadow<[X10_64, X12_64, X14_64, X16_64],
                                               [X11_64, X13_64, X15_64, X17_64]>>,

  CCIfType<[i64, f64], CCAssignToStack<8, 8>>
]>;


def CSR_RV32 : CalleeSavedRegs<(add X1_32, X3_32, X4_32, X8_32, X9_32,
                               (sequence "X%u_32", 18, 27))>;

def CSR_RV64 : CalleeSavedRegs<(add X1_64, X3_64, X4_64, X8_64, X9_64,
                               (sequence "X%u_64", 18, 27))>;

// Needed for implementation of RISCVRegisterInfo::getNoPreservedMask()
def CSR_NoRegs : CalleeSavedRegs<(add)>;
