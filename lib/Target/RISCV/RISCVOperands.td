//===-- RISCVOperands.td - RISCV instruction operands --------*- tblgen-*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

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

def uimm5 : Operand<iPTR>, ImmLeaf<iPTR, [{return isUInt<5>(Imm);}]> {
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

//===----------------------------------------------------------------------===//
// Immediate asm operands.
//===----------------------------------------------------------------------===//

class ImmediateAsmOperand<string name>
  : AsmOperandClass {
  let Name = name;
  //let RenderMethod = "addImmOperands";
}

def S12Imm : ImmediateAsmOperand<"S12Imm">;

//===----------------------------------------------------------------------===//
// 64-bit immediates
//===----------------------------------------------------------------------===//

// Constructs both a DAG pattern and instruction operand for an immediate
// of type VT.  PRED returns true if a node is acceptable and XFORM returns
// the operand value associated with the node.  ASMOP is the name of the
// associated asm operand, and also forms the basis of the asm print method.
class Immediate<ValueType vt, code pred, SDNodeXForm xform, string asmop>
  : PatLeaf<(vt imm), pred, xform>, Operand<vt> {
  let PrintMethod = "print"##asmop##"Operand";
  //let ParserMatchClass = !cast<AsmOperandClass>(asmop);
}

//sign-extended 12 bit immediate
def imm64sx12 : Immediate<i64, [{
  return isInt<12>(N->getSExtValue());
}], NOOP_SDNodeXForm, "S12Imm">;

//===----------------------------------------------------------------------===//
// Load/Store Operands
//

class MemOperand : Operand<i32> { let OperandType = "OPERAND_MEMORY"; }

def memreg : Operand<i32> {
  let MIOperandInfo = (ops GPR);
  //let EncoderMethod = "getMemRegEncoding";
  let OperandType = "OPERAND_MEMORY";
  let PrintMethod = "printMemRegOperand";
}

def regaddr : ComplexPattern<iPTR, 1, "selectRegAddr">;

// addr_reg_imm12s := [reg + imm12s]
def AddrRegImm12sAsmOperand : AsmOperandClass { let Name = "AddrRegImm12s"; }
def addr_reg_imm12s : MemOperand,
  ComplexPattern<iPTR, 2, "SelectAddrRegImm12s", []> {
  // 12-bit signed immediate operand.
  // Immediate access range would be +/- (imm12s << 1).

  let MIOperandInfo = (ops ptr_rc, iPTR:$offsimm);
  let PrintMethod = "printAddrRegImmOperand";
  // Parser function not completed yet. Finish it later.
  let ParserMatchClass = AddrRegImm12sAsmOperand;
}
