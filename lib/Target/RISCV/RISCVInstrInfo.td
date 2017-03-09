//===-- RISCVInstrInfo.td - Target Description for RISCV ---*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file describes the RISC-V instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

include "RISCVInstrFormats.td"

def SDT_RISCVCallSeqStart : SDCallSeqStart<[SDTCisVT<0, i32>]>;
def SDT_RISCVCallSeqEnd   : SDCallSeqEnd<[SDTCisVT<0, i32>,
                                         SDTCisVT<1, i32>]>;
def SDT_RISCVCall         : SDTypeProfile<0, -1, [SDTCisVT<0, i32>]>;
def SDT_RISCVSelectCC     : SDTypeProfile<1, 5, [SDTCisSameAs<1, 2>,
                                                 SDTCisSameAs<0, 4>,
                                                 SDTCisSameAs<4, 5>]>;

def SDT_RISCVStructByVal : SDTypeProfile<0, 4,
                                         [SDTCisVT<0, i32>, SDTCisVT<1, i32>,
                                          SDTCisVT<2, i32>, SDTCisVT<3, i32>]>;

def Call             : SDNode<"RISCVISD::CALL", SDT_RISCVCall,
                              [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue,
                               SDNPVariadic]>;
def RetFlag : SDNode<"RISCVISD::RET_FLAG", SDTNone,
                     [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;
def CallSeqStart     : SDNode<"ISD::CALLSEQ_START", SDT_RISCVCallSeqStart,
                              [SDNPHasChain, SDNPOutGlue]>;
def CallSeqEnd       : SDNode<"ISD::CALLSEQ_END", SDT_RISCVCallSeqEnd,
                              [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;
def SelectCC         : SDNode<"RISCVISD::SELECT_CC", SDT_RISCVSelectCC, [SDNPInGlue]>;

def RISCVcopystructbyval : SDNode<"RISCVISD::COPY_STRUCT_BYVAL" ,
                                   SDT_RISCVStructByVal,
                                  [SDNPHasChain, SDNPInGlue, SDNPOutGlue,
                                   SDNPMayStore, SDNPMayLoad]>;


// Operands
class ImmAsmOperand<string prefix, int width, string suffix> : AsmOperandClass {
  let Name = prefix # "Imm" # width # suffix;
  let RenderMethod = "addImmOperands";
  let DiagnosticType = !strconcat("Invalid", Name);
}

class SImmAsmOperand<int width, string suffix = "">
  : ImmAsmOperand<"S", width, suffix> {
}

class UImmAsmOperand<int width, string suffix = "">
  : ImmAsmOperand<"U", width, suffix> {
}

def uimm4 : Operand<i32> {
  let ParserMatchClass = UImmAsmOperand<4>;
  let DecoderMethod = "decodeUImmOperand<4>";
}

def uimm5 : Operand<i32>, ImmLeaf<i32, [{return isUInt<5>(Imm);}]> {
  let ParserMatchClass = UImmAsmOperand<5>;
  let DecoderMethod = "decodeUImmOperand<5>";
}

def simm12 : Operand<i32>, ImmLeaf<i32, [{return isInt<12>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<12>;
  let EncoderMethod = "getImmOpValue";
  let DecoderMethod = "decodeSImmOperand<12>";
}

def uimm12 : Operand<i32> {
  let ParserMatchClass = UImmAsmOperand<12>;
  let DecoderMethod = "decodeUImmOperand<12>";
}

// A 13-bit signed immediate where the least significant bit is zero.
def simm13_lsb0 : Operand<OtherVT> {
  let ParserMatchClass = SImmAsmOperand<13, "Lsb0">;
  let EncoderMethod = "getImmOpValueAsr1";
  let DecoderMethod = "decodeSImmOperandAndLsl1<13>";
}

def uimm20 : Operand<i32> {
  let ParserMatchClass = UImmAsmOperand<20>;
  let EncoderMethod = "getImmOpValue";
  let DecoderMethod = "decodeUImmOperand<20>";
}

// A 21-bit signed immediate where the least significant bit is zero.
def simm21_lsb0 : Operand<OtherVT> {
  let ParserMatchClass = SImmAsmOperand<21, "Lsb0">;
  let EncoderMethod = "getImmOpValueAsr1";
  let DecoderMethod = "decodeSImmOperandAndLsl1<21>";
}

// Extract least significant 12 bits from an immediate value and sign extend
// them.
def LO12Sext : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(SignExtend64<12>(N->getZExtValue()),
                                   SDLoc(N), MVT::i32);
}]>;

// Extract the most significant 20 bits from an immediate value. Add 1 if bit
// 11 is 1, to compensate for the low 12 bits in the matching immediate addi
// or ld/st being negative.
def HI20 : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(((N->getZExtValue()+0x800) >> 12) & 0xfffff,
                                   SDLoc(N), MVT::i32);
}]>;


// As noted in RISCVRegisterInfo.td, the hope is that support for
// variable-sized register classes will mean that instruction definitions do
// not need to be duplicated for 32-bit and 64-bit register classes. For now
// we use 'GPR', which is 32-bit. When codegen for both RV32 and RV64 is
// added, we will need to duplicate instruction definitions unless a proposal
// like <http://lists.llvm.org/pipermail/llvm-dev/2016-September/105027.html>
// is adopted.

def LUI : FU<0b0110111, (outs GPR:$rd), (ins uimm20:$imm20),
             "lui\t$rd, $imm20", []>;

def AUIPC : FU<0b0010111, (outs GPR:$rd), (ins uimm20:$imm20),
             "auipc\t$rd, $imm20", []>;

let isCall=1 in {
def JAL : FUJ<0b1101111, (outs GPR:$rd), (ins simm21_lsb0:$imm20),
              "jal\t$rd, $imm20", []>;
}

let isTerminator=1, isBarrier=1 in {
def PseudoBR : Pseudo<(outs), (ins simm21_lsb0:$imm20), [(br bb:$imm20)]>,
               PseudoInstExpansion<(JAL X0_32, simm21_lsb0:$imm20)>;
}

let isCall=1 in {
def JALR : FI<0b000, 0b1100111, (outs GPR:$rd), (ins GPR:$rs1, simm12:$imm12),
              "jalr\t$rd, $rs1, $imm12", []>;
}

let isBranch = 1, isBarrier = 1, isTerminator = 1, isIndirectBranch = 1 in {
  def PseudoBRIND : Pseudo<(outs), (ins GPR:$rs1), [(brind GPR:$rs1)]>,
                    PseudoInstExpansion<(JALR X0_32, GPR:$rs1, 0)>;
}

let isCall=1, Defs=[X1_32] in {
  def PseudoCALL : Pseudo<(outs), (ins GPR:$rs1), [(Call GPR:$rs1)]>,
                   PseudoInstExpansion<(JALR X1_32, GPR:$rs1, 0)>;
}

let isReturn=1, isTerminator=1, isBarrier=1 in {
  def PseudoRET : Pseudo<(outs), (ins), [(RetFlag)]>,
                  PseudoInstExpansion<(JALR X0_32, X1_32, 0)>;
}

// Pessimstically assume the stack pointer will be clobbered
let Defs = [X2_32], Uses = [X2_32] in {
  def ADJCALLSTACKDOWN : Pseudo<(outs), (ins i32imm:$amt),
                                [(CallSeqStart timm:$amt)]>;
  def ADJCALLSTACKUP   : Pseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                                [(CallSeqEnd timm:$amt1, timm:$amt2)]>;
}

class Bcc<bits<3> funct3, string OpcodeStr, PatFrag CondOp> :
      FSB<funct3, 0b1100011, (outs), (ins GPR:$rs1, GPR:$rs2, simm13_lsb0:$imm12),
         OpcodeStr#"\t$rs1, $rs2, $imm12", [(brcond (i32 (CondOp GPR:$rs1, GPR:$rs2)), bb:$imm12)]> {
  let isBranch = 1;
  let isTerminator = 1;
}

def BEQ   : Bcc<0b000, "beq", seteq>;
def BNE   : Bcc<0b001, "bne", setne>;
def BLT   : Bcc<0b100, "blt", setlt>;
def BGE   : Bcc<0b101, "bge", setge>;
def BLTU  : Bcc<0b110, "bltu", setult>;
def BGEU  : Bcc<0b111, "bgeu", setuge>;

class Bcc_SwapPat<PatFrag CondOp, RISCVInst InstBcc> : Pat<
  (brcond (i32 (CondOp GPR:$rs1, GPR:$rs2)), bb:$imm12),
  (InstBcc GPR:$rs2, GPR:$rs1, bb:$imm12)>;

// Condition codes that don't have matching RISC-V branch instructions, but
// are trivially supported by swapping the two input operands
def : Bcc_SwapPat<setgt, BLT>;
def : Bcc_SwapPat<setle, BGE>;
def : Bcc_SwapPat<setugt, BLTU>;
def : Bcc_SwapPat<setule, BGEU>;

def : Pat<(brcond GPR:$cond, bb:$imm12), (BNE GPR:$cond, X0_32, bb:$imm12)>;

let usesCustomInserter = 1 in {
  def Select : Pseudo<(outs GPR:$dst),
                      (ins GPR:$lhs, GPR:$rhs, i32imm:$imm, GPR:$src, GPR:$src2),
                      [(set i32:$dst,
                       (SelectCC GPR:$lhs, GPR:$rhs, (i32 imm:$imm), GPR:$src, GPR:$src2))]>;
}

class LD_ri<bits<3> funct3, string OpcodeStr> :
      FI<funct3, 0b0000011, (outs GPR:$rd), (ins GPR:$rs1, simm12:$imm12),
         OpcodeStr#"\t$rd, ${imm12}(${rs1})", []> {
  let mayLoad = 1;
}

def LB   : LD_ri<0b000, "lb">;
def LH   : LD_ri<0b001, "lh">;
def LW   : LD_ri<0b010, "lw">;
def LBU  : LD_ri<0b100, "lbu">;
def LHU  : LD_ri<0b101, "lhu">;

multiclass LD_Pat<PatFrag LoadOp, RISCVInst Inst> {
  def : Pat<(LoadOp GPR:$rs1), (Inst GPR:$rs1, 0)>;
  def : Pat<(LoadOp (add GPR:$rs1, simm12:$imm12)), (Inst GPR:$rs1, simm12:$imm12)>;
}

defm : LD_Pat<sextloadi8, LB>;
defm : LD_Pat<extloadi8, LB>;
defm : LD_Pat<sextloadi16, LH>;
defm : LD_Pat<extloadi16, LH>;
defm : LD_Pat<load, LW>;
defm : LD_Pat<zextloadi8, LBU>;
defm : LD_Pat<zextloadi16, LHU>;

// Operands for stores are in the order srcreg, base, offset rather than
// reflecting the order these fields are specified in the instruction
// encoding.

class ST_ri<bits<3> funct3, string OpcodeStr> :
      FS<funct3, 0b0100011, (outs), (ins GPR:$rs2, GPR:$rs1, simm12:$imm12),
         OpcodeStr#"\t$rs2, ${imm12}(${rs1})", []> {
  let mayStore = 1;
}

multiclass ST_Pat<PatFrag StoreOp, RISCVInst Inst> {
  def : Pat<(StoreOp GPR:$rs2, GPR:$rs1), (Inst GPR:$rs2, GPR:$rs1, 0)>;
  def : Pat<(StoreOp GPR:$rs2, (add GPR:$rs1, simm12:$imm12)), (Inst GPR:$rs2, GPR:$rs1, simm12:$imm12)>;
}

def SB  : ST_ri<0b000, "sb">;
def SH  : ST_ri<0b001, "sh">;
def SW  : ST_ri<0b010, "sw">;

defm : ST_Pat<truncstorei8, SB>;
defm : ST_Pat<truncstorei16, SH>;
defm : ST_Pat<store, SW>;

class ALU_ri<bits<3> funct3, string OpcodeStr, SDPatternOperator OpNode> :
      FI<funct3, 0b0010011, (outs GPR:$rd), (ins GPR:$rs1, simm12:$imm12),
         OpcodeStr#"\t$rd, $rs1, $imm12", [(set GPR:$rd, (OpNode GPR:$rs1, simm12:$imm12))]>
{
}

def ADDI  : ALU_ri<0b000, "addi", add>;
def SLTI  : ALU_ri<0b010, "slti", setlt>;
def SLTIU : ALU_ri<0b011, "sltiu", setult>;
def XORI  : ALU_ri<0b100, "xori", xor>;
def ORI   : ALU_ri<0b110, "ori", or>;
def ANDI  : ALU_ri<0b111, "andi", and>;

class SHIFT32_ri<bit arithshift, bits<3> funct3, string OpcodeStr, SDPatternOperator OpNode> :
      FI32Shift<arithshift, funct3, 0b0010011, (outs GPR:$rd), (ins GPR:$rs1, uimm5:$shamt),
         OpcodeStr#"\t$rd, $rs1, $shamt", [(set GPR:$rd, (OpNode GPR:$rs1, uimm5:$shamt))]>
{
}

def SLLI : SHIFT32_ri<0, 0b001, "slli", shl>;
def SRLI : SHIFT32_ri<0, 0b101, "srli", srl>;
def SRAI : SHIFT32_ri<1, 0b101, "srai", sra>;

class ALU_rr<bits<7> funct7, bits<3> funct3, string OpcodeStr, SDPatternOperator OpNode> :
      FR<funct7, funct3, 0b0110011, (outs GPR:$rd), (ins GPR:$rs1, GPR:$rs2),
         OpcodeStr#"\t$rd, $rs1, $rs2", [(set GPR:$rd, (OpNode GPR:$rs1, GPR:$rs2))]>
{
}

def ADD  : ALU_rr<0b0000000, 0b000, "add", add>;
def SUB  : ALU_rr<0b0100000, 0b000, "sub", sub>;
def SLL  : ALU_rr<0b0000000, 0b001, "sll", shl>;
def SLT  : ALU_rr<0b0000000, 0b010, "slt", setlt>;
def SLTU : ALU_rr<0b0000000, 0b011, "sltu", setult>;
def XOR  : ALU_rr<0b0000000, 0b100, "xor", xor>;
def SRL  : ALU_rr<0b0000000, 0b101, "srl", srl>;
def SRA  : ALU_rr<0b0100000, 0b101, "sra", sra>;
def OR   : ALU_rr<0b0000000, 0b110, "or", or>;
def AND  : ALU_rr<0b0000000, 0b111, "and", and>;

// Define pattern expansions for setcc operations that aren't directly
// handled by a RISC-V instruction

def : Pat<(seteq GPR:$rs1, GPR:$rs2), (SLTIU (XOR GPR:$rs1, GPR:$rs2), 1)>;
def : Pat<(setne GPR:$rs1, GPR:$rs2), (SLTU X0_32, (XOR GPR:$rs1, GPR:$rs2))>;
def : Pat<(setugt GPR:$rs1, GPR:$rs2), (SLTU GPR:$rs2, GPR:$rs1)>;
def : Pat<(setuge GPR:$rs1, GPR:$rs2), (XORI (SLTU GPR:$rs1, GPR:$rs2), 1)>;
def : Pat<(setule GPR:$rs1, GPR:$rs2), (XORI (SLTU GPR:$rs2, GPR:$rs1), 1)>;
def : Pat<(setgt GPR:$rs1, GPR:$rs2), (SLT GPR:$rs2, GPR:$rs1)>;
def : Pat<(setge GPR:$rs1, GPR:$rs2), (XORI (SLT GPR:$rs1, GPR:$rs2), 1)>;
def : Pat<(setle GPR:$rs1, GPR:$rs2), (XORI (SLT GPR:$rs2, GPR:$rs1), 1)>;

def FENCE : FI<0b000, 0b0001111, (outs), (ins uimm4:$pred, uimm4:$succ),
               "fence\t$pred, $succ", []> {
  bits<4> pred;
  bits<4> succ;

  let rs1 = 0;
  let rd = 0;
  let imm12 = {0b0000,pred,succ};
}

def FENCEI : FI<0b001, 0b0001111, (outs), (ins), "fence.i", []> {
  let rs1 = 0;
  let rd = 0;
  let imm12 = 0;
}

let rs1=0, rd=0 in {
  def ECALL  : FI<0b000, 0b1110011, (outs), (ins), "ecall", []> {
    let imm12=0;
  }
  def EBREAK : FI<0b000, 0b1110011, (outs), (ins), "ebreak", []> {
    let imm12=1;
  }
}

class CSR_rr<bits<3> funct3, string OpcodeStr> :
      FI<funct3, 0b1110011, (outs GPR:$rd), (ins uimm12:$imm12, GPR:$rs1),
         OpcodeStr#"\t$rd, $imm12, $rs1", []>
{
}

def CSRRW : CSR_rr<0b001, "csrrw">;
def CSRRS : CSR_rr<0b010, "csrrs">;
def CSRRC : CSR_rr<0b011, "csrrc">;

class CSR_ri<bits<3> funct3, string OpcodeStr> :
      FI<funct3, 0b1110011, (outs GPR:$rd), (ins uimm12:$imm12, uimm5:$rs1),
         OpcodeStr#"\t$rd, $imm12, $rs1", []>
{
}

def CSRRWI : CSR_ri<0b101, "csrrwi">;
def CSRRSI : CSR_ri<0b110, "csrrsi">;
def CSRRCI : CSR_ri<0b111, "csrrci">;

let usesCustomInserter = 1 in {
    def COPY_STRUCT_BYVAL_I32 : Pseudo<
      (outs), (ins GPR:$dst, GPR:$src, i32imm:$size, i32imm:$alignment),
      [(RISCVcopystructbyval GPR:$dst, GPR:$src, imm:$size, imm:$alignment)]>;
}

// signed 12-bit immediate
def : Pat<(simm12:$imm), (ADDI X0_32, simm12:$imm)>;

// 32-bit immediate
def : Pat<(i32 imm:$imm), (ADDI (LUI (HI20 imm:$imm)), (LO12Sext imm:$imm))>;
