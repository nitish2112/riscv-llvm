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

class SLT_rr<bits<7> funct7, bits<3> funct3, string OpcodeStr, SDPatternOperator OpNode> :
      FR<funct7, funct3, 0b0110011, (outs GPR:$rd), (ins GPR64:$rs1, GPR64:$rs2),
         OpcodeStr#"\t$rd, $rs1, $rs2", [(set GPR:$rd, (OpNode GPR64:$rs1, GPR64:$rs2))]>;

def SLT32  : SLT_rr<0b0000000, 0b010, "slt",  setlt>,  Requires<[IsRV64]>;
def SLTU32 : SLT_rr<0b0000000, 0b011, "sltu", setult>, Requires<[IsRV64]>;

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

class SLT_ri<bits<3> funct3, string OpcodeStr, SDPatternOperator OpNode> :
      FI<funct3, 0b0010011, (outs GPR:$rd), (ins GPR64:$rs1, imm64sx12:$imm12),
         OpcodeStr#"\t$rd, $rs1, $imm12", [(set GPR:$rd, (OpNode GPR64:$rs1, imm64sx12:$imm12))]>;

def SLTI32 : SLT_ri<0b010, "slti",  setlt>,  Requires<[IsRV64]>;
def SLTIU32: SLT_ri<0b011, "sltiu", setult>, Requires<[IsRV64]>;

// signed 12-bit immediate
def : RV64Pat<(imm64sx12:$imm), (ADDI64 X0_64, imm64sx12:$imm)>;

def : RV64Pat<(seteq GPR64:$rs1, GPR64:$rs2), (SLTIU64 (XOR64 GPR64:$rs1, GPR64:$rs2), 1)>;
def : RV64Pat<(seteq GPR64:$rs1, GPR64:$rs2), (SLTIU32 (XOR64 GPR64:$rs1, GPR64:$rs2), 1)>;
def : RV64Pat<(setne GPR64:$rs1, GPR64:$rs2), (SLTU64 X0_64, (XOR64 GPR64:$rs1, GPR64:$rs2))>;
def : RV64Pat<(setne GPR64:$rs1, GPR64:$rs2), (SLTU32 X0_64, (XOR64 GPR64:$rs1, GPR64:$rs2))>;
def : RV64Pat<(setugt GPR64:$rs1, GPR64:$rs2), (SLTU64 GPR64:$rs2, GPR64:$rs1)>;
def : RV64Pat<(setugt GPR64:$rs1, GPR64:$rs2), (SLTU32 GPR64:$rs2, GPR64:$rs1)>;
def : RV64Pat<(setuge GPR64:$rs1, GPR64:$rs2), (XORI64 (SLTU64 GPR64:$rs1, GPR64:$rs2), 1)>;
def : RV64Pat<(setule GPR64:$rs1, GPR64:$rs2), (XORI64 (SLTU64 GPR64:$rs2, GPR64:$rs1), 1)>;
def : RV64Pat<(setgt GPR64:$rs1, GPR64:$rs2), (SLT64 GPR64:$rs2, GPR64:$rs1)>;
def : RV64Pat<(setgt GPR64:$rs1, GPR64:$rs2), (SLT32 GPR64:$rs2, GPR64:$rs1)>;
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

//===----------------------------------------------------------------------===//
// Load and Store Instructions
//===----------------------------------------------------------------------===//

let mayLoad = 1 in {
  def LWU : InstLoad <"lwu" , 0b0000011, 0b110, zextloadi32,  GPR64>, Requires<[IsRV64]>;
  def LD  : InstLoad <"ld"  , 0b0000011, 0b011, load,  GPR64>, Requires<[IsRV64]>;
  def LW64_32 : InstLoad <"lw" , 0b0000011, 0b010, load, GPR>, Requires<[IsRV64]>;
  def LH64_32 : InstLoad <"lh" , 0b0000011, 0b001, sextloadi16, GPR>, Requires<[IsRV64]>;
  def LHU64_32: InstLoad <"lhu", 0b0000011, 0b101, zextloadi16, GPR>, Requires<[IsRV64]>;
  def LB64_32 : InstLoad <"lb" , 0b0000011, 0b000, sextloadi8, GPR>, Requires<[IsRV64]>;
  def LBU64_32: InstLoad <"lbu", 0b0000011, 0b100, zextloadi8, GPR>, Requires<[IsRV64]>;
  def LW64 : InstLoad <"lw" , 0b0000011, 0b010, sextloadi32, GPR64>, Requires<[IsRV64]>;
  def LH64 : InstLoad <"lh" , 0b0000011, 0b001, sextloadi16, GPR64>, Requires<[IsRV64]>;
  def LHU64: InstLoad <"lhu", 0b0000011, 0b101, zextloadi16, GPR64>, Requires<[IsRV64]>;
  def LB64 : InstLoad <"lb" , 0b0000011, 0b000, sextloadi8, GPR64>, Requires<[IsRV64]>;
  def LBU64: InstLoad <"lbu", 0b0000011, 0b100, zextloadi8, GPR64>, Requires<[IsRV64]>;
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
  def SD : InstStore <"sd"  , 0b0100011, 0b011, store, GPR64>, Requires<[IsRV64]>;
  def SW64 : InstStore<"sw" , 0b0100011, 0b010, truncstorei32, GPR64>, Requires<[IsRV64]>;
  def SH64 : InstStore<"sh" , 0b0100011, 0b001, truncstorei16, GPR64>, Requires<[IsRV64]>;
  def SB64 : InstStore<"sb" , 0b0100011, 0b000, truncstorei8 , GPR64>, Requires<[IsRV64]>;
  def SW64_32 : InstStore<"sw" , 0b0100011, 0b010, store, GPR>, Requires<[IsRV64]>;
  def SH64_32 : InstStore<"sh" , 0b0100011, 0b001, truncstorei16, GPR>, Requires<[IsRV64]>;
  def SB64_32 : InstStore<"sb" , 0b0100011, 0b000, truncstorei8 , GPR>, Requires<[IsRV64]>;
}

//===----------------------------------------------------------------------===//
// Branch and Call Instructions
//===----------------------------------------------------------------------===//

let isCall=1 in {
def JAL64 : FUJ<0b1101111, (outs GPR64:$rd), (ins simm21_lsb0:$imm20),
                "jal\t$rd, $imm20", []>;
}

let isBranch = 1, isTerminator=1, isBarrier=1 in {
def PseudoBR64 : Pseudo<(outs), (ins simm21_lsb0:$imm20), [(br bb:$imm20)]>,
                 PseudoInstExpansion<(JAL64 X0_64, simm21_lsb0:$imm20)>,
                 Requires<[IsRV64]>;
}

let isCall=1 in {
def JALR64 : FI<0b000, 0b1100111, (outs GPR64:$rd),
                (ins GPR64:$rs1, simm12:$imm12),
                "jalr\t$rd, $rs1, $imm12", []>;
}

let isBranch = 1, isBarrier = 1, isTerminator = 1, isIndirectBranch = 1 in {
  def PseudoBRIND64 : Pseudo<(outs), (ins GPR64:$rs1), [(brind GPR64:$rs1)]>,
                      PseudoInstExpansion<(JALR64 X0_64, GPR64:$rs1, 0)>,
                      Requires<[IsRV64]>;
}

class Bcc64<bits<3> funct3, string OpcodeStr, PatFrag CondOp> :
      FSB<funct3, 0b1100011, (outs),
          (ins GPR64:$rs1, GPR64:$rs2, simm13_lsb0:$imm12),
          OpcodeStr#"\t$rs1, $rs2, $imm12",
          [(brcond (i32 (CondOp GPR64:$rs1, GPR64:$rs2)), bb:$imm12)]> {
  let isBranch = 1;
  let isTerminator = 1;
}

def BEQ64   : Bcc64<0b000, "beq", seteq>;
def BNE64   : Bcc64<0b001, "bne", setne>;
def BLT64   : Bcc64<0b100, "blt", setlt>;
def BGE64   : Bcc64<0b101, "bge", setge>;
def BLTU64  : Bcc64<0b110, "bltu", setult>;
def BGEU64  : Bcc64<0b111, "bgeu", setuge>;

class Bcc_SwapPat64<PatFrag CondOp, RISCVInst InstBcc> : Pat<
  (brcond (i32 (CondOp GPR64:$rs1, GPR64:$rs2)), bb:$imm12),
  (InstBcc GPR64:$rs2, GPR64:$rs1, bb:$imm12)>;

// Condition codes that don't have matching RISC-V branch instructions, but
// are trivially supported by swapping the two input operands
def : Bcc_SwapPat64<setgt, BLT64>;
def : Bcc_SwapPat64<setle, BGE64>;
def : Bcc_SwapPat64<setugt, BLTU64>;
def : Bcc_SwapPat64<setule, BGEU64>;

def : Pat<(brcond GPR64:$cond, bb:$imm12),
          (BNE64 GPR64:$cond, X0_64, bb:$imm12)>;

let isCall=1, Defs=[X1_64] in {
  def PseudoCALL64 : Pseudo<(outs), (ins GPR64:$rs1), [(Call GPR64:$rs1)]>,
                     PseudoInstExpansion<(JALR64 X1_64, GPR64:$rs1, 0)>,
                     Requires<[IsRV64]>;
}

let isReturn=1, isTerminator=1, isBarrier=1 in {
  def PseudoRET64 : Pseudo<(outs), (ins), [(RetFlag)]>,
                    PseudoInstExpansion<(JALR64 X0_64, X1_64, 0)>,
                    Requires<[IsRV64]>;
}

// Get i32 value from GPR64 register
def :Pat<(i32 (trunc GPR64:$src)), (EXTRACT_SUBREG GPR64:$src, sub_32)>;

// Extension patterns
def : Pat<(i64 (zext GPR:$val)), (SUBREG_TO_REG (i64 0), GPR:$val, sub_32)>;
def : Pat<(i64 (sext GPR:$val)), (SUBREG_TO_REG (i64 0), GPR:$val, sub_32)>;
def : Pat<(i64 (anyext GPR:$val)), (SUBREG_TO_REG (i64 0), GPR:$val, sub_32)>;

def LUI64 : FU<0b0110111, (outs GPR64:$rd), (ins imm64sxu20:$imm20),
               "lui\t$rd, $imm20", []>;
