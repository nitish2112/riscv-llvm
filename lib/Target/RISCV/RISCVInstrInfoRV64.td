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
def SLT64  : ALU64_rr<0b0000000, 0b010, "slt",  setlt>,  Requires<[IsRV64]>;
def SLTU64 : ALU64_rr<0b0000000, 0b011, "sltu", setult>, Requires<[IsRV64]>;

class ALU64_ri<bits<3> funct3, string OpcodeStr, SDPatternOperator OpNode> :
      FI<funct3, 0b0010011, (outs GPR64:$rd), (ins GPR64:$rs1, imm64sx12:$imm12),
         OpcodeStr#"\t$rd, $rs1, $imm12", [(set GPR64:$rd, (OpNode GPR64:$rs1, imm64sx12:$imm12))]>;

//Integer arithmetic register-immediate
def ADDI64: ALU64_ri<0b000, "addi", add>, Requires<[IsRV64]>;
def XORI64: ALU64_ri<0b100, "xori", xor>, Requires<[IsRV64]>;
def ORI64 : ALU64_ri<0b110,  "ori",  or>, Requires<[IsRV64]>;
def ANDI64: ALU64_ri<0b111, "andi", and>, Requires<[IsRV64]>;
def SLTI64 : ALU64_ri<0b010, "slti",  setlt>,  Requires<[IsRV64]>;
def SLTIU64: ALU64_ri<0b011, "sltiu", setult>, Requires<[IsRV64]>;

def : RV64Pat<(seteq GPR64:$rs1, GPR64:$rs2), (SLTIU64 (XOR64 GPR64:$rs1, GPR64:$rs2), 1)>;
def : RV64Pat<(setne GPR64:$rs1, GPR64:$rs2), (SLTU64 X0_64, (XOR64 GPR64:$rs1, GPR64:$rs2))>;
def : RV64Pat<(setugt GPR64:$rs1, GPR64:$rs2), (SLTU64 GPR64:$rs2, GPR64:$rs1)>;
def : RV64Pat<(setuge GPR64:$rs1, GPR64:$rs2), (XORI64 (SLTU64 GPR64:$rs1, GPR64:$rs2), 1)>;
def : RV64Pat<(setule GPR64:$rs1, GPR64:$rs2), (XORI64 (SLTU64 GPR64:$rs2, GPR64:$rs1), 1)>;
def : RV64Pat<(setgt GPR64:$rs1, GPR64:$rs2), (SLT64 GPR64:$rs2, GPR64:$rs1)>;
def : RV64Pat<(setge GPR64:$rs1, GPR64:$rs2), (XORI64 (SLT64 GPR64:$rs1, GPR64:$rs2), 1)>;
def : RV64Pat<(setle GPR64:$rs1, GPR64:$rs2), (XORI64 (SLT64 GPR64:$rs2, GPR64:$rs1), 1)>;

class SHIFT64_ri<bit arithshift, bits<3> funct3, string OpcodeStr, SDPatternOperator OpNode> :
      FI32Shift<arithshift, funct3, 0b0010011, (outs GPR64:$rd), (ins GPR64:$rs1, uimm5:$shamt),
         OpcodeStr#"\t$rd, $rs1, $shamt", [(set GPR64:$rd, (OpNode GPR64:$rs1, uimm5:$shamt))]>;

def SLLI64: SHIFT64_ri<0, 0b001 , "slli", shl>, Requires<[IsRV64]>;
def SRLI64: SHIFT64_ri<0, 0b101 , "srli", srl>, Requires<[IsRV64]>;
def SRAI64: SHIFT64_ri<1, 0b101 , "srai", sra>, Requires<[IsRV64]>;

class SHIFTW_ri<bit arithshift, bits<3> funct3, string OpcodeStr, SDPatternOperator OpNode> :
      FI32Shift<arithshift, funct3, 0b0011011, (outs GPR:$rd), (ins GPR:$rs1, uimm5:$shamt),
         OpcodeStr#"\t$rd, $rs1, $shamt", [(set GPR:$rd, (OpNode GPR:$rs1, uimm5:$shamt))]>;

def SLLIW: SHIFTW_ri<0, 0b001 , "slliw", shl>, Requires<[IsRV64]>;
def SRLIW: SHIFTW_ri<0, 0b101 , "srliw", srl>, Requires<[IsRV64]>;
def SRAIW: SHIFTW_ri<1, 0b101 , "sraiw", sra>, Requires<[IsRV64]>;

//Load/Store Instructions
let mayLoad = 1 in {
  def LWU : InstLoad <"lwu" , 0b0000011, 0b110, zextloadi32,  GPR64, mem64>, Requires<[IsRV64]>;
  def LD  : InstLoad <"ld"  , 0b0000011, 0b011, load,  GPR64, mem64>, Requires<[IsRV64]>;
  def LW64_32 : InstLoad <"lw" , 0b0000011, 0b010, load, GPR, mem64>, Requires<[IsRV64]>;
  def LH64_32 : InstLoad <"lh" , 0b0000011, 0b001, sextloadi16, GPR, mem64>, Requires<[IsRV64]>;
  def LHU64_32: InstLoad <"lhu", 0b0000011, 0b101, zextloadi16, GPR, mem64>, Requires<[IsRV64]>;
  def LB64_32 : InstLoad <"lb" , 0b0000011, 0b000, sextloadi8, GPR, mem64>, Requires<[IsRV64]>;
  def LBU64_32: InstLoad <"lbu", 0b0000011, 0b100, zextloadi8, GPR, mem64>, Requires<[IsRV64]>;
  def LW64 : InstLoad <"lw" , 0b0000011, 0b010, sextloadi32, GPR64, mem64>, Requires<[IsRV64]>;
  def LH64 : InstLoad <"lh" , 0b0000011, 0b001, sextloadi16, GPR64, mem64>, Requires<[IsRV64]>;
  def LHU64: InstLoad <"lhu", 0b0000011, 0b101, zextloadi16, GPR64, mem64>, Requires<[IsRV64]>;
  def LB64 : InstLoad <"lb" , 0b0000011, 0b000, sextloadi8, GPR64, mem64>, Requires<[IsRV64]>;
  def LBU64: InstLoad <"lbu", 0b0000011, 0b100, zextloadi8, GPR64, mem64>, Requires<[IsRV64]>;
}
//extended loads
def : Pat<(i64 (extloadi1  addr_reg_imm12s:$addr)), (LBU64 addr_reg_imm12s:$addr)>;
def : Pat<(i32 (extloadi1  addr_reg_imm12s:$addr)), (LBU64_32 addr_reg_imm12s:$addr)>;
def : Pat<(extloadi1  addr_reg_imm12s:$addr), (LBU64 addr_reg_imm12s:$addr)>;
def : Pat<(i64 (extloadi8  addr_reg_imm12s:$addr)), (LBU64 addr_reg_imm12s:$addr)>;
def : Pat<(i32 (extloadi8  addr_reg_imm12s:$addr)), (LBU64_32 addr_reg_imm12s:$addr)>;
def : Pat<(extloadi8  addr_reg_imm12s:$addr), (LBU64 addr_reg_imm12s:$addr)>;
def : Pat<(i64 (extloadi16 addr_reg_imm12s:$addr)), (LHU64 addr_reg_imm12s:$addr)>;
def : Pat<(i32 (extloadi16 addr_reg_imm12s:$addr)), (LHU64_32 addr_reg_imm12s:$addr)>;
def : Pat<(extloadi16 addr_reg_imm12s:$addr), (LHU64 addr_reg_imm12s:$addr)>;
def : Pat<(i64 (extloadi32 addr_reg_imm12s:$addr)), (LW64   addr_reg_imm12s:$addr)>;
def : Pat<(extloadi32 addr_reg_imm12s:$addr), (LW64   addr_reg_imm12s:$addr)>;
//def : RV64Pat<(i32 (extloadi1  addr_reg_imm12s:$addr)), (LBU64_32 addr_reg_imm12s:$addr)>;
def : RV64Pat<(i32 (extloadi8  addr_reg_imm12s:$addr)), (LBU64_32 addr_reg_imm12s:$addr)>;
//def : RV64Pat<(i32 (extloadi16 addr_reg_imm12s:$addr)), (LHU64_32 addr_reg_imm12s:$addr)>;

let mayStore = 1 in {
  def SD : InstStore <"sd"  , 0b0100011, 0b011, store, GPR64, mem64>, Requires<[IsRV64]>;
  def SW64 : InstStore<"sw" , 0b0100011, 0b010, truncstorei32, GPR64, mem64>, Requires<[IsRV64]>;
  def SH64 : InstStore<"sh" , 0b0100011, 0b001, truncstorei16, GPR64, mem64>, Requires<[IsRV64]>;
  def SB64 : InstStore<"sb" , 0b0100011, 0b000, truncstorei8 , GPR64, mem64>, Requires<[IsRV64]>;
  def SW64_32 : InstStore<"sw" , 0b0100011, 0b010, store, GPR, mem64>, Requires<[IsRV64]>;
  def SH64_32 : InstStore<"sh" , 0b0100011, 0b001, truncstorei16, GPR, mem64>, Requires<[IsRV64]>;
  def SB64_32 : InstStore<"sb" , 0b0100011, 0b000, truncstorei8 , GPR, mem64>, Requires<[IsRV64]>;
}
