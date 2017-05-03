//===- RISCVInstrInfoRV64.td - RISCV RV64I instructions -------*- tblgen-*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

class ALUW_rr<bits<7> funct7, bits<3> funct3, string OpcodeStr, SDPatternOperator OpNode> :
      FR<funct7, funct3, 0b0111011, (outs GPR:$rd), (ins GPR:$rs1, GPR:$rs2),
         OpcodeStr#"\t$rd, $rs1, $rs2", [(set GPR:$rd, (OpNode GPR:$rs1, GPR:$rs2))]>;

def ADDW  : ALUW_rr<0b0000000, 0b000, "addw", add>, Requires<[IsRV64]>;
def SUBW  : ALUW_rr<0b0100000, 0b000, "subw", sub>, Requires<[IsRV64]>;
def SLLW  : ALUW_rr<0b0000000, 0b001, "sllw", shl>, Requires<[IsRV64]>;
def SRLW  : ALUW_rr<0b0000000, 0b101, "srlw", srl>, Requires<[IsRV64]>;
def SRAW  : ALUW_rr<0b0100000, 0b101, "sraw", sra>, Requires<[IsRV64]>;


class ALUW_ri<bits<3> funct3, string OpcodeStr, SDPatternOperator OpNode> :
      FI<funct3, 0b0011011, (outs GPR:$rd), (ins GPR:$rs1, simm12:$imm12),
         OpcodeStr#"\t$rd, $rs1, $imm12", [(set GPR:$rd, (OpNode GPR:$rs1, simm12:$imm12))]>;

//Integer arithmetic register-immediate
def ADDIW  : ALUW_ri<0b101, "addiw", add>, Requires<[IsRV64]>;


