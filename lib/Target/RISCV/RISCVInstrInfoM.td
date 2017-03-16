//===- RISCVInstrInfoM.td - Multiply Divide RISCV instructions -*-tblgen-*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

def MUL   : ALU_rr<0b0000001, 0b000, "mul",  mul>;
def MULH  : ALU_rr<0b0000001, 0b001, "mulh", mulhs>;
def MULHU : ALU_rr<0b0000001, 0b011, "mulhu", mulhu>;
def DIV   : ALU_rr<0b0000001, 0b100, "div",  sdiv>;
def DIVU  : ALU_rr<0b0000001, 0b101, "divu", udiv>;
def REM   : ALU_rr<0b0000001, 0b110, "rem",  srem>;
def REMU  : ALU_rr<0b0000001, 0b111, "remu", urem>;
