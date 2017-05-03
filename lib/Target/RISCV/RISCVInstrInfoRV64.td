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

class ALU64_rr<bits<7> funct7, bits<3> funct3, string OpcodeStr, SDPatternOperator OpNode> :
      FR<funct7, funct3, 0b0110011, (outs GPR64:$rd), (ins GPR64:$rs1, GPR64:$rs2),
         OpcodeStr#"\t$rd, $rs1, $rs2", [(set GPR64:$rd, (OpNode GPR64:$rs1, GPR64:$rs2))]>;

//Standard instructions operating on 64bit values
//Integer arithmetic register-register
def ADD64  : ALU64_rr<0b0000000, 0b000, "add", add>, Requires<[IsRV64]>;
def SUB64  : ALU64_rr<0b0100000, 0b000, "sub", sub>, Requires<[IsRV64]>;
def SLL64  : ALU64_rr<0b0000000, 0b001, "sll", shl>, Requires<[IsRV64]>;
def XOR64  : ALU64_rr<0b0000000, 0b100, "xor", xor>, Requires<[IsRV64]>;
def SRL64  : ALU64_rr<0b0000000, 0b101, "srl", srl>, Requires<[IsRV64]>;
def SRA64  : ALU64_rr<0b0100000, 0b101, "sra", sra>, Requires<[IsRV64]>;
def OR64   : ALU64_rr<0b0000000, 0b110, "or" , or>,  Requires<[IsRV64]>;
def AND64  : ALU64_rr<0b0000000, 0b111, "and", and>, Requires<[IsRV64]>;

class ALU64_ri<bits<3> funct3, string OpcodeStr, SDPatternOperator OpNode> :
      FI<funct3, 0b0010011, (outs GPR64:$rd), (ins GPR64:$rs1, imm64sx12:$imm12),
         OpcodeStr#"\t$rd, $rs1, $imm12", [(set GPR64:$rd, (OpNode GPR64:$rs1, imm64sx12:$imm12))]>;

//Integer arithmetic register-immediate
def ADDI64: ALU64_ri<0b000, "addi", add>, Requires<[IsRV64]>;
def XORI64: ALU64_ri<0b100, "xori", xor>, Requires<[IsRV64]>;
def ORI64 : ALU64_ri<0b110,  "ori",  or>, Requires<[IsRV64]>;
def ANDI64: ALU64_ri<0b111, "andi", and>, Requires<[IsRV64]>;
