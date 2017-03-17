//===- RISCVInstrInfoM.td - Multiply Divide RISCV instructions -*-tblgen-*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

def MUL   : ALU_rr<0b0000001, 0b000, "mul",  mul>, Requires<[IsRV32, HasM]>;
def MULH  : ALU_rr<0b0000001, 0b001, "mulh", mulhs>, Requires<[HasM]>;
def MULHU : ALU_rr<0b0000001, 0b011, "mulhu", mulhu>, Requires<[HasM]>;
def DIV   : ALU_rr<0b0000001, 0b100, "div",  sdiv>, Requires<[IsRV32, HasM]>;
def DIVU  : ALU_rr<0b0000001, 0b101, "divu", udiv>, Requires<[IsRV32, HasM]>;
def REM   : ALU_rr<0b0000001, 0b110, "rem",  srem>, Requires<[IsRV32, HasM]>;
def REMU  : ALU_rr<0b0000001, 0b111, "remu", urem>, Requires<[IsRV32, HasM]>;
