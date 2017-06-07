//===-- RISCVAsmParser.cpp - Parse RISCV assembly to MCInst instructions --===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/RISCVMCTargetDesc.h"
#include "MCTargetDesc/RISCVMCExpr.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/None.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

namespace {
struct RISCVOperand;

class RISCVAsmParser : public MCTargetAsmParser {
  SMLoc getLoc() const { return getParser().getTok().getLoc(); }

  bool generateImmOutOfRangeError(OperandVector &Operands, uint64_t ErrorInfo,
                                  int Lower, int Upper, Twine Msg);

  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;

  Optional<unsigned> matchCPURegisterName(StringRef Symbol) const;

  bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc, SMLoc &EndLoc) override;

  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;

  bool ParseDirective(AsmToken DirectiveID) override;

// Auto-generated instruction matching functions
#define GET_ASSEMBLER_HEADER
#include "RISCVGenAsmMatcher.inc"

  bool parseLeftParen(bool report_error);
  bool parseRightParen(bool report_error);
  Optional<StringRef> peekIdentifier();
  Optional<StringRef> parseIdentifier(bool report_error);
  Optional<unsigned> parseRegister(bool report_error);
  Optional<const MCExpr*> parseImmediate(bool report_error);

  bool parseOperand(OperandVector &Operands);

public:
  enum RISCVMatchResultTy {
    Match_Dummy = FIRST_TARGET_MATCH_RESULT_TY,
#define GET_OPERAND_DIAGNOSTIC_TYPES
#include "RISCVGenAsmMatcher.inc"
#undef GET_OPERAND_DIAGNOSTIC_TYPES
  };

  static bool classifySymbolRef(const MCExpr *Expr,
                                RISCVMCExpr::VariantKind &Kind,
                                int64_t &Addend);

  RISCVAsmParser(const MCSubtargetInfo &STI, MCAsmParser &Parser,
                 const MCInstrInfo &MII, const MCTargetOptions &Options)
      : MCTargetAsmParser(Options, STI) {
    setAvailableFeatures(ComputeAvailableFeatures(STI.getFeatureBits()));
  }
};

/// RISCVOperand - Instances of this class represent a parsed machine
/// instruction
struct RISCVOperand : public MCParsedAsmOperand {

  enum KindTy {
    Token,
    Register,
    Memory,
    Immediate,
  } Kind;

  struct RegOp {
    unsigned RegNum;
  };

  struct ImmOp {
    const MCExpr *Val;
  };

  SMLoc StartLoc, EndLoc;
  union {
    StringRef Tok;
    RegOp Reg;
    ImmOp Imm;
  };

  RISCVOperand(KindTy K) : MCParsedAsmOperand(), Kind(K) {}

public:
  RISCVOperand(const RISCVOperand &o) : MCParsedAsmOperand() {
    Kind = o.Kind;
    StartLoc = o.StartLoc;
    EndLoc = o.EndLoc;
    switch (Kind) {
    case Register:
      Reg = o.Reg;
      break;
    case Immediate:
      Imm = o.Imm;
      break;
    case Token:
      Tok = o.Tok;
      break;
    case Memory:
      //Todo
      break;
    }
  }

  bool isToken() const override { return Kind == Token; }
  bool isReg() const override { return Kind == Register; }
  bool isImm() const override { return Kind == Immediate; }
  bool isMem() const override { return Kind == Memory; }

  bool isConstantImm() const {
    return isImm() && dyn_cast<MCConstantExpr>(getImm());
  }

  int64_t getConstantImm() const {
    const MCExpr *Val = getImm();
    return static_cast<const MCConstantExpr *>(Val)->getValue();
  }

  bool isGPRAsmReg() const {
    return isReg();
  }

  // Predicate methods for AsmOperands defined in RISCVInstrInfo.td

  bool isUImm4() const {
    return (isConstantImm() && isUInt<4>(getConstantImm()));
  }

  bool isUImm5() const {
    return (isConstantImm() && isUInt<5>(getConstantImm()));
  }

  bool isUImm6() const {
    return (isConstantImm() && isUInt<6>(getConstantImm()));
  }

  bool isS12Imm() const {
    return isSImm12();
  }

  bool isSImm12() const {
    if (isConstantImm()) {
      return isInt<12>(getConstantImm());
    } else if (isImm()) {
      RISCVMCExpr::VariantKind VK;
      int64_t Addend;
      if (!RISCVAsmParser::classifySymbolRef(getImm(), VK, Addend))
        return false;
      return VK == RISCVMCExpr::VK_RISCV_LO;
    }
    return false;
  }

  bool isUImm12() const {
    return (isConstantImm() && isUInt<12>(getConstantImm()));
  }

  bool isSImm13Lsb0() const {
    if (isConstantImm()) {
      return isShiftedInt<12, 1>(getConstantImm());
    } else if (isImm()) {
      RISCVMCExpr::VariantKind VK;
      int64_t Addend;
      return RISCVAsmParser::classifySymbolRef(getImm(), VK, Addend);
    }
    return false;
  }

  bool isUImm20() const {
    if (isConstantImm()) {
      return isUInt<20>(getConstantImm());
    } else if (isImm()) {
      RISCVMCExpr::VariantKind VK;
      int64_t Addend;
      if (!RISCVAsmParser::classifySymbolRef(getImm(), VK, Addend))
        return false;
      return VK == RISCVMCExpr::VK_RISCV_HI || VK == RISCVMCExpr::VK_RISCV_PCREL_HI;
    }
    return false;
  }

  // Reg + imm12s
  bool isAddrRegImm12s() const {
    // ToDo
    return false;
  }

  bool isSImm21Lsb0() const {
    if (isConstantImm()) {
      return isShiftedInt<20, 1>(getConstantImm());
    } else if (isImm()) {
      RISCVMCExpr::VariantKind VK;
      int64_t Addend;
      return RISCVAsmParser::classifySymbolRef(getImm(), VK, Addend);
    }
    return false;
  }

  /// getStartLoc - Gets location of the first token of this operand
  SMLoc getStartLoc() const override { return StartLoc; }
  /// getEndLoc - Gets location of the last token of this operand
  SMLoc getEndLoc() const override { return EndLoc; }

  unsigned getReg() const override {
    assert(Kind == Register && "Invalid type access!");
    return Reg.RegNum;
  }

  const MCExpr *getImm() const {
    assert(Kind == Immediate && "Invalid type access!");
    return Imm.Val;
  }

  StringRef getToken() const {
    assert(Kind == Token && "Invalid type access!");
    return Tok;
  }

  void print(raw_ostream &OS) const override {
    switch (Kind) {
    case Immediate:
      OS << *getImm();
      break;
    case Register:
      OS << "<register x";
      OS << getReg() << ">";
      break;
    case Token:
      OS << "'" << getToken() << "'";
      break;
    case Memory:
      // Todo
      break;
    }
  }

  static std::unique_ptr<RISCVOperand> createToken(StringRef Str, SMLoc S) {
    auto Op = make_unique<RISCVOperand>(Token);
    Op->Tok = Str;
    Op->StartLoc = S;
    Op->EndLoc = S;
    return Op;
  }

  static std::unique_ptr<RISCVOperand> createReg(unsigned RegNo, SMLoc S,
                                                 SMLoc E) {
    auto Op = make_unique<RISCVOperand>(Register);
    Op->Reg.RegNum = RegNo;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static std::unique_ptr<RISCVOperand> createImm(const MCExpr *Val, SMLoc S,
                                                 SMLoc E, MCContext &Ctx) {
    auto Op = make_unique<RISCVOperand>(Immediate);
    if (const RISCVMCExpr *RE = dyn_cast<RISCVMCExpr>(Val)) {
      int64_t Res;
      if (RE->evaluateAsConstant(Res))
        Val = MCConstantExpr::create(Res, Ctx);
    }
    Op->Imm.Val = Val;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  void addExpr(MCInst &Inst, const MCExpr *Expr) const {
    assert(Expr && "Expr shouldn't be null!");
    if (auto *CE = dyn_cast<MCConstantExpr>(Expr))
      Inst.addOperand(MCOperand::createImm(CE->getValue()));
    else
      Inst.addOperand(MCOperand::createExpr(Expr));
  }

  // Used by the TableGen Code
  void addRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    addExpr(Inst, getImm());
  }

  void addAddrRegImm12sOperands(MCInst &Inst, unsigned N) const {
    // Todo
  }
};
} // end anonymous namespace.

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#define GET_SUBTARGET_FEATURE_NAME
#include "RISCVGenAsmMatcher.inc"

bool RISCVAsmParser::generateImmOutOfRangeError(
    OperandVector &Operands, uint64_t ErrorInfo, int Lower, int Upper,
    Twine Msg = "immediate must be an integer in the range") {
  SMLoc ErrorLoc = ((RISCVOperand &)*Operands[ErrorInfo]).getStartLoc();
  return Error(ErrorLoc, Msg + " [" + Twine(Lower) + ", " + Twine(Upper) + "]");
}

bool RISCVAsmParser::MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                                             OperandVector &Operands,
                                             MCStreamer &Out,
                                             uint64_t &ErrorInfo,
                                             bool MatchingInlineAsm) {
  MCInst Inst;

  switch (MatchInstructionImpl(Operands, Inst, ErrorInfo, MatchingInlineAsm)) {
  default:
    break;
  case Match_Success:
    Inst.setLoc(IDLoc);
    Out.EmitInstruction(Inst, getSTI());
    return false;
  case Match_MissingFeature:
    return Error(IDLoc, "instruction use requires an option to be enabled");
  case Match_MnemonicFail:
    return Error(IDLoc, "unrecognized instruction mnemonic");
  case Match_InvalidOperand: {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0U) {
      if (ErrorInfo >= Operands.size())
        return Error(ErrorLoc, "too few operands for instruction");

      ErrorLoc = ((RISCVOperand &)*Operands[ErrorInfo]).getStartLoc();
      if (ErrorLoc == SMLoc())
        ErrorLoc = IDLoc;
    }
    return Error(ErrorLoc, "invalid operand for instruction");
  }
  case Match_InvalidUImm4:
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 4) - 1);
  case Match_InvalidUImm5:
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 5) - 1);
  case Match_InvalidSImm12:
    return generateImmOutOfRangeError(Operands, ErrorInfo, -(1 << 11),
                                      (1 << 11) - 1);
  case Match_InvalidUImm12:
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 12) - 1);
  case Match_InvalidSImm13Lsb0:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 12), (1 << 12) - 2,
        "immediate must be a multiple of 2 bytes in the range");
  case Match_InvalidUImm20:
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 20) - 1);
  case Match_InvalidSImm21Lsb0:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 20), (1 << 20) - 2,
        "immediate must be a multiple of 2 bytes in the range");
  }

  llvm_unreachable("Unknown match type detected!");
}

Optional<unsigned> RISCVAsmParser::matchCPURegisterName(StringRef Name) const {
  if (getSTI().getFeatureBits()[RISCV::FeatureRV32]) {
      return StringSwitch<Optional<unsigned>>(Name)
               .Cases("zero", "x0" , Optional<unsigned>(RISCV::X0_32 ))
               .Cases("ra"  , "x1" , Optional<unsigned>(RISCV::X1_32 ))
               .Cases("sp"  , "x2" , Optional<unsigned>(RISCV::X2_32 ))
               .Cases("gp"  , "x3" , Optional<unsigned>(RISCV::X3_32 ))
               .Cases("tp"  , "x4" , Optional<unsigned>(RISCV::X4_32 ))
               .Cases("t0"  , "x5" , Optional<unsigned>(RISCV::X5_32 ))
               .Cases("t1"  , "x6" , Optional<unsigned>(RISCV::X6_32 ))
               .Cases("t2"  , "x7" , Optional<unsigned>(RISCV::X7_32 ))
               .Cases("s0"  , "x8" , Optional<unsigned>(RISCV::X8_32 ))
               .Cases("s1"  , "x9" , Optional<unsigned>(RISCV::X9_32 ))
               .Cases("a0"  , "x10", Optional<unsigned>(RISCV::X10_32))
               .Cases("a1"  , "x11", Optional<unsigned>(RISCV::X11_32))
               .Cases("a2"  , "x12", Optional<unsigned>(RISCV::X12_32))
               .Cases("a3"  , "x13", Optional<unsigned>(RISCV::X13_32))
               .Cases("a4"  , "x14", Optional<unsigned>(RISCV::X14_32))
               .Cases("a5"  , "x15", Optional<unsigned>(RISCV::X15_32))
               .Cases("a6"  , "x16", Optional<unsigned>(RISCV::X16_32))
               .Cases("a7"  , "x17", Optional<unsigned>(RISCV::X17_32))
               .Cases("s2"  , "x18", Optional<unsigned>(RISCV::X18_32))
               .Cases("s3"  , "x19", Optional<unsigned>(RISCV::X19_32))
               .Cases("s4"  , "x20", Optional<unsigned>(RISCV::X20_32))
               .Cases("s5"  , "x21", Optional<unsigned>(RISCV::X21_32))
               .Cases("s6"  , "x22", Optional<unsigned>(RISCV::X22_32))
               .Cases("s7"  , "x23", Optional<unsigned>(RISCV::X23_32))
               .Cases("s8"  , "x24", Optional<unsigned>(RISCV::X24_32))
               .Cases("s9"  , "x25", Optional<unsigned>(RISCV::X25_32))
               .Cases("s10" , "x26", Optional<unsigned>(RISCV::X26_32))
               .Cases("s11" , "x27", Optional<unsigned>(RISCV::X27_32))
               .Cases("t3"  , "x28", Optional<unsigned>(RISCV::X28_32))
               .Cases("t4"  , "x29", Optional<unsigned>(RISCV::X29_32))
               .Cases("t5"  , "x30", Optional<unsigned>(RISCV::X30_32))
               .Cases("t6"  , "x31", Optional<unsigned>(RISCV::X31_32))
               .Default(None);
  } else if (getSTI().getFeatureBits()[RISCV::FeatureRV64]) {
      return StringSwitch<Optional<unsigned>>(Name)
               .Cases("zero", "x0" , Optional<unsigned>(RISCV::X0_64 ))
               .Cases("ra"  , "x1" , Optional<unsigned>(RISCV::X1_64 ))
               .Cases("sp"  , "x2" , Optional<unsigned>(RISCV::X2_64 ))
               .Cases("gp"  , "x3" , Optional<unsigned>(RISCV::X3_64 ))
               .Cases("tp"  , "x4" , Optional<unsigned>(RISCV::X4_64 ))
               .Cases("t0"  , "x5" , Optional<unsigned>(RISCV::X5_64 ))
               .Cases("t1"  , "x6" , Optional<unsigned>(RISCV::X6_64 ))
               .Cases("t2"  , "x7" , Optional<unsigned>(RISCV::X7_64 ))
               .Cases("s0"  , "x8" , Optional<unsigned>(RISCV::X8_64 ))
               .Cases("s1"  , "x9" , Optional<unsigned>(RISCV::X9_64 ))
               .Cases("a0"  , "x10", Optional<unsigned>(RISCV::X10_64))
               .Cases("a1"  , "x11", Optional<unsigned>(RISCV::X11_64))
               .Cases("a2"  , "x12", Optional<unsigned>(RISCV::X12_64))
               .Cases("a3"  , "x13", Optional<unsigned>(RISCV::X13_64))
               .Cases("a4"  , "x14", Optional<unsigned>(RISCV::X14_64))
               .Cases("a5"  , "x15", Optional<unsigned>(RISCV::X15_64))
               .Cases("a6"  , "x16", Optional<unsigned>(RISCV::X16_64))
               .Cases("a7"  , "x17", Optional<unsigned>(RISCV::X17_64))
               .Cases("s2"  , "x18", Optional<unsigned>(RISCV::X18_64))
               .Cases("s3"  , "x19", Optional<unsigned>(RISCV::X19_64))
               .Cases("s4"  , "x20", Optional<unsigned>(RISCV::X20_64))
               .Cases("s5"  , "x21", Optional<unsigned>(RISCV::X21_64))
               .Cases("s6"  , "x22", Optional<unsigned>(RISCV::X22_64))
               .Cases("s7"  , "x23", Optional<unsigned>(RISCV::X23_64))
               .Cases("s8"  , "x24", Optional<unsigned>(RISCV::X24_64))
               .Cases("s9"  , "x25", Optional<unsigned>(RISCV::X25_64))
               .Cases("s10" , "x26", Optional<unsigned>(RISCV::X26_64))
               .Cases("s11" , "x27", Optional<unsigned>(RISCV::X27_64))
               .Cases("t3"  , "x28", Optional<unsigned>(RISCV::X28_64))
               .Cases("t4"  , "x29", Optional<unsigned>(RISCV::X29_64))
               .Cases("t5"  , "x30", Optional<unsigned>(RISCV::X30_64))
               .Cases("t6"  , "x31", Optional<unsigned>(RISCV::X31_64))
               .Default(None);
  } else {
      llvm_unreachable("Not RV32/64");
  }
}

bool RISCVAsmParser::ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                   SMLoc &EndLoc) {
  const auto StartLoc_ = getParser().getTok().getLoc();
  const auto EndLoc_ = getParser().getTok().getEndLoc();
  auto OptReg = parseRegister(true);
  if (OptReg) {
    RegNo = *OptReg;
    StartLoc = StartLoc_;
    EndLoc = EndLoc_;
    return false;
  } else {
    return true;
  }
}

bool RISCVAsmParser::parseLeftParen(bool report_error) {
  if (getLexer().is(AsmToken::LParen)) {
    getLexer().Lex();
    return true;
  } else {
    if (report_error) Error(getLoc(), "expected '('");
    return false;
  }
}

bool RISCVAsmParser::parseRightParen(bool report_error) {
  if (getLexer().is(AsmToken::RParen)) {
    getLexer().Lex();
    return true;
  } else {
    if (report_error) Error(getLoc(), "expected ')'");
    return false;
  }
}

Optional<StringRef> RISCVAsmParser::peekIdentifier() {
  if (getLexer().is(AsmToken::Identifier))
    return Optional<StringRef>(getLexer().getTok().getIdentifier());
  else
    return None;
}

Optional<StringRef> RISCVAsmParser::parseIdentifier(bool report_error) {
  const Optional<StringRef> Identifier = peekIdentifier();
  if (!Identifier) {
    if (report_error) Error(getLoc(), "expected identifier");
  } else {
    getLexer().Lex();
  }

  return Identifier;
}

Optional<unsigned> RISCVAsmParser::parseRegister(bool report_error) {
  const Optional<StringRef> Identifier = peekIdentifier();
  if (Identifier) {
    const Optional<unsigned> Reg = matchCPURegisterName(*Identifier);
    if (Reg) {
        getLexer().Lex();
        return Reg;
    } else {
        if (report_error) Error(getLoc(), "invalid register");
        return None;
    }
  } else {
    if (report_error) Error(getLoc(), "expected register");
    return None;
  }
}

Optional<const MCExpr*> RISCVAsmParser::parseImmediate(bool report_error) {
  switch (getLexer().getKind()) {
    case AsmToken::LParen:
    case AsmToken::Minus:
    case AsmToken::Plus:
    case AsmToken::Integer:
    case AsmToken::String: {
      const MCExpr* Expr;
      if (getParser().parseExpression(Expr))
        return None;
      else
        return Optional<const MCExpr*>(Expr);
    }
    case AsmToken::Identifier: {
      const Optional<StringRef> Identifier = parseIdentifier(true);
      if (Identifier) {
        MCSymbol* Sym = getContext().getOrCreateSymbol(*Identifier);
        return Optional<const MCExpr*>(MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext()));
      } else {
        return None;
      }
    }
    case AsmToken::Percent: {
      getLexer().Lex(); // Eat '%'
      const Optional<StringRef> Identifier = peekIdentifier();
      if (Identifier) {
        const RISCVMCExpr::VariantKind VK = RISCVMCExpr::getVariantKindForName(*Identifier);
        if (VK == RISCVMCExpr::VK_RISCV_None) {
          if (report_error) Error(getLoc(), "unrecognized operand modifier");
          return None;
        }
        getLexer().Lex(); // Eat identifier

        if (!parseLeftParen(true)) return None;
        const MCExpr* SubExpr;
        if (getParser().parseExpression(SubExpr)) {
          if (report_error) Error(getLoc(), "expected expression");
          return None;
        }
        if (!parseRightParen(true)) return None;

        return Optional<const MCExpr*>(RISCVMCExpr::create(SubExpr, VK, getContext()));
      } else {
        return None;
      }
    }
    default:
      if (report_error) Error(getLoc(), "expected immediate");
      return None;
  }
}

/// Looks at a token type and creates the relevant operand
/// from this information, adding to Operands.
/// If operand was parsed, returns false, else true.
bool RISCVAsmParser::parseOperand(OperandVector &Operands) {
  const SMLoc S = getLoc();

  const auto Reg = parseRegister(false);
  if (Reg) {
    Operands.push_back(RISCVOperand::createReg(*Reg, S, getLoc()));
    return false;
  }

  const auto Expr = parseImmediate(true);
  if (Expr) {
    if (dyn_cast<MCConstantExpr>(*Expr) && parseLeftParen(false)) { // offset(reg)
      const int64_t Imm = dyn_cast<MCConstantExpr>(*Expr)->getValue();
      const auto Reg = parseRegister(true);
      if (Reg) {
        if (!parseRightParen(true)) return true;
        // TODO: createMem
        return false;
      } else {
        return true;
      }
    } else {
      Operands.push_back(RISCVOperand::createImm(*Expr, S, getLoc(), getContext()));
      return false;
    }
  }

  return true;
}

bool RISCVAsmParser::ParseInstruction(ParseInstructionInfo &Info,
                                      StringRef Name, SMLoc NameLoc,
                                      OperandVector &Operands) {
  // First operand is token for instruction
  Operands.push_back(RISCVOperand::createToken(Name, NameLoc));

  // If there are no more operands, then finish
  if (getLexer().is(AsmToken::EndOfStatement))
    return false;

  // Parse first operand
  if (parseOperand(Operands))
    return true;

  // Parse until end of statement, consuming commas between operands
  while (getLexer().is(AsmToken::Comma)) {
    // Consume comma token
    getLexer().Lex();

    // Parse next operand
    if (parseOperand(Operands))
      return true;
  }

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    SMLoc Loc = getLexer().getLoc();
    getParser().eatToEndOfStatement();
    return Error(Loc, "unexpected token");
  }

  getParser().Lex(); // Consume the EndOfStatement.
  return false;
}

bool RISCVAsmParser::classifySymbolRef(const MCExpr *Expr,
                                       RISCVMCExpr::VariantKind &Kind,
                                       int64_t &Addend) {
  Kind = RISCVMCExpr::VK_RISCV_Invalid;
  Addend = 0;

  if (const RISCVMCExpr *RE = dyn_cast<RISCVMCExpr>(Expr)) {
    Kind = RE->getKind();
    Expr = RE->getSubExpr();
  }

  assert(!isa<MCConstantExpr>(Expr) &&
         "simple constants should have been evaluated already");

  const MCSymbolRefExpr *SE = dyn_cast<MCSymbolRefExpr>(Expr);
  if (SE) {
    // It's a simple symbol reference with no addend.
    return true;
  }

  const MCBinaryExpr *BE = dyn_cast<MCBinaryExpr>(Expr);
  if (!BE)
    return false;

  SE = dyn_cast<MCSymbolRefExpr>(BE->getLHS());
  if (!SE)
    return false;

  if (BE->getOpcode() != MCBinaryExpr::Add &&
      BE->getOpcode() != MCBinaryExpr::Sub)
    return false;

  // We are able to support the subtraction of two symbol references
  if (BE->getOpcode() == MCBinaryExpr::Sub &&
      isa<MCSymbolRefExpr>(BE->getLHS()) && isa<MCSymbolRefExpr>(BE->getRHS()))
    return true;

  // See if the addend is is a constant, otherwise there's more going
  // on here than we can deal with.
  auto AddendExpr = dyn_cast<MCConstantExpr>(BE->getRHS());
  if (!AddendExpr)
    return false;

  Addend = AddendExpr->getValue();
  if (BE->getOpcode() == MCBinaryExpr::Sub)
    Addend = -Addend;

  // It's some symbol reference + a constant addend
  return Kind != RISCVMCExpr::VK_RISCV_Invalid;
}

bool RISCVAsmParser::ParseDirective(AsmToken DirectiveID) { return true; }

extern "C" void LLVMInitializeRISCVAsmParser() {
  RegisterMCAsmParser<RISCVAsmParser> X(getTheRISCV32Target());
  RegisterMCAsmParser<RISCVAsmParser> Y(getTheRISCV64Target());
}
