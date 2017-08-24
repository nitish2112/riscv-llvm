//===-- RISCVCallingConv.cpp - Calling conventions for RISCV --------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "RISCVCallingConv.h"

#define GET_REGINFO_ENUM
#include "RISCVGenRegisterInfo.inc"

using namespace llvm;

const MCPhysReg RISCV::RV32ArgGPRs[RISCV::NumArgRV32GPRs] = {
    RISCV::X10_32, RISCV::X11_32, RISCV::X12_32, RISCV::X13_32,
    RISCV::X14_32, RISCV::X15_32, RISCV::X16_32, RISCV::X17_32
};
