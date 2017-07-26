//===-- RISCVDisassembler.cpp - Disassembler for RISCV --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the RISCVDisassembler class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/RISCVMCTargetDesc.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCFixedLenDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Endian.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "riscv-disassembler"

typedef MCDisassembler::DecodeStatus DecodeStatus;

namespace {
class RISCVDisassembler : public MCDisassembler {

public:
  RISCVDisassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
      : MCDisassembler(STI, Ctx) {}

  bool isRV64() const { return STI.getFeatureBits()[RISCV::FeatureRV64]; }

  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                              ArrayRef<uint8_t> Bytes, uint64_t Address,
                              raw_ostream &VStream,
                              raw_ostream &CStream) const override;
};
} // end anonymous namespace

static MCDisassembler *createRISCVDisassembler(const Target &T,
                                               const MCSubtargetInfo &STI,
                                               MCContext &Ctx) {
  return new RISCVDisassembler(STI, Ctx);
}

extern "C" void LLVMInitializeRISCVDisassembler() {
  // Register the disassembler for each target.
  TargetRegistry::RegisterMCDisassembler(getTheRISCV32Target(),
                                         createRISCVDisassembler);
  TargetRegistry::RegisterMCDisassembler(getTheRISCV64Target(),
                                         createRISCVDisassembler);
}

static const unsigned GPRDecoderTable[] = {
  RISCV::X0_32,  RISCV::X1_32,  RISCV::X2_32,  RISCV::X3_32,
  RISCV::X4_32,  RISCV::X5_32,  RISCV::X6_32,  RISCV::X7_32,
  RISCV::X8_32,  RISCV::X9_32,  RISCV::X10_32, RISCV::X11_32,
  RISCV::X12_32, RISCV::X13_32, RISCV::X14_32, RISCV::X15_32,
  RISCV::X16_32, RISCV::X17_32, RISCV::X18_32, RISCV::X19_32,
  RISCV::X20_32, RISCV::X21_32, RISCV::X22_32, RISCV::X23_32,
  RISCV::X24_32, RISCV::X25_32, RISCV::X26_32, RISCV::X27_32,
  RISCV::X28_32, RISCV::X29_32, RISCV::X30_32, RISCV::X31_32
};

static const unsigned GPR64DecoderTable[] = {
  RISCV::X0_64,  RISCV::X1_64,  RISCV::X2_64,  RISCV::X3_64,
  RISCV::X4_64,  RISCV::X5_64,  RISCV::X6_64,  RISCV::X7_64,
  RISCV::X8_64,  RISCV::X9_64,  RISCV::X10_64, RISCV::X11_64,
  RISCV::X12_64, RISCV::X13_64, RISCV::X14_64, RISCV::X15_64,
  RISCV::X16_64, RISCV::X17_64, RISCV::X18_64, RISCV::X19_64,
  RISCV::X20_64, RISCV::X21_64, RISCV::X22_64, RISCV::X23_64,
  RISCV::X24_64, RISCV::X25_64, RISCV::X26_64, RISCV::X27_64,
  RISCV::X28_64, RISCV::X29_64, RISCV::X30_64, RISCV::X31_64
};


static DecodeStatus DecodeGPRRegisterClass(MCInst &Inst, uint64_t RegNo,
                                           uint64_t Address,
                                           const void *Decoder) {
   if (RegNo > sizeof(GPRDecoderTable)) {
     return MCDisassembler::Fail;
   }

   // We must define our own mapping from RegNo to register identifier.
   // Accessing index RegNo in the register class will work in the case that
   // registers were added in ascending order, but not in general.
   unsigned Reg = GPRDecoderTable[RegNo];
   Inst.addOperand(MCOperand::createReg(Reg));
   return MCDisassembler::Success;
}

static DecodeStatus DecodeGPR64RegisterClass(MCInst &Inst,
                                             unsigned RegNo,
                                             uint64_t Address,
                                             const void *Decoder) {
  if (RegNo > 31)
    return MCDisassembler::Fail;

  unsigned Reg = GPR64DecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

template <unsigned N>
static DecodeStatus decodeUImmOperand(MCInst &Inst, uint64_t Imm,
                                      int64_t Address, const void *Decoder) {
  assert(isUInt<N>(Imm) && "Invalid immediate");
  Inst.addOperand(MCOperand::createImm(Imm));
  return MCDisassembler::Success;
}

template <unsigned N>
static DecodeStatus decodeSImmOperand(MCInst &Inst, uint64_t Imm,
                                      int64_t Address, const void *Decoder) {
  assert(isUInt<N>(Imm) && "Invalid immediate");
  // Sign-extend the number in the bottom N bits of Imm
  Inst.addOperand(MCOperand::createImm(SignExtend64<N>(Imm)));
  return MCDisassembler::Success;
}

template <unsigned N>
static DecodeStatus decodeSImmOperandAndLsl1(MCInst &Inst, uint64_t Imm,
                                             int64_t Address,
                                             const void *Decoder) {
  assert(isUInt<N>(Imm) && "Invalid immediate");
  // Sign-extend the number in the bottom N bits of Imm after accounting for
  // the fact that the N bit immediate is stored in N-1 bits (the LSB is
  // always zero)
  Inst.addOperand(MCOperand::createImm(SignExtend64<N>(Imm << 1)));
  return MCDisassembler::Success;
}

static bool isLoad(unsigned Opc) {
  if (Opc == RISCV::LB  || Opc == RISCV::LB64  ||
      Opc == RISCV::LBU || Opc == RISCV::LBU64 ||
      Opc == RISCV::LD  || Opc == RISCV::LH    ||
      Opc == RISCV::LHU || Opc == RISCV::LHU64 ||
      Opc == RISCV::LW  || Opc == RISCV::LW64 ||
      Opc == RISCV::LWU || Opc == RISCV::LH64)
    return true;
  return false;
}

static DecodeStatus decodeAddrRegImm(MCInst &Inst, unsigned Insn,
                                     uint64_t Address, const void *Decoder);

static DecodeStatus decodeAddrSpImm6uWord(MCInst &Inst, unsigned Insn,
                                          uint64_t Address,
                                          const void *Decoder);

#include "RISCVGenDisassemblerTables.inc"

DecodeStatus RISCVDisassembler::getInstruction(MCInst &MI, uint64_t &Size,
                                               ArrayRef<uint8_t> Bytes,
                                               uint64_t Address,
                                               raw_ostream &OS,
                                               raw_ostream &CS) const {
  // TODO: although assuming 4-byte instructions is sufficient for RV32 and
  // RV64, this will need modification when supporting the compressed
  // instruction set extension (RVC) which uses 16-bit instructions. Other
  // instruction set extensions have the option of defining instructions up to
  // 176 bits wide.
  Size = 4;
  if (Bytes.size() < 4) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  // Get the four bytes of the instruction.
  uint32_t Inst = support::endian::read32le(Bytes.data());

  if (isRV64())
    return decodeInstruction(DecoderTableRISCV64_32, MI, Inst, Address, this, STI);
  else
    return decodeInstruction(DecoderTable32, MI, Inst, Address, this, STI);
}

static DecodeStatus decodeAddrRegImm(MCInst &Inst,
                                     unsigned Insn,
                                     uint64_t Address,
                                     const void *Decoder) {
  int32_t Imm, Reg;

  if (isLoad(Inst.getOpcode())) {
    Imm = fieldFromInstruction(Insn, 5, 12);
    Reg = fieldFromInstruction(Insn, 0, 5);
  } else {
    Reg = fieldFromInstruction(Insn, 5, 5);
    Imm = fieldFromInstruction(Insn, 0, 5) | fieldFromInstruction(Insn, 10, 7) << 5;
  }

  if (static_cast<const RISCVDisassembler *>(Decoder)->isRV64())
    DecodeGPR64RegisterClass(Inst, Reg, Address, Decoder);
  else
    DecodeGPRRegisterClass(Inst, Reg, Address, Decoder);

  Inst.addOperand(MCOperand::createImm(SignExtend64<12>(Imm)));

  return MCDisassembler::Success;
}

static DecodeStatus decodeAddrSpImm6uWord(MCInst &Inst,
                                          unsigned Insn,
                                          uint64_t Address,
                                          const void *Decoder) {
  int32_t Imm, Reg = 2;

  Imm = fieldFromInstruction(Insn, 0, 0) << 2;

  if (static_cast<const RISCVDisassembler *>(Decoder)->isRV64())
    DecodeGPR64RegisterClass(Inst, Reg, Address, Decoder);
  else
    DecodeGPRRegisterClass(Inst, Reg, Address, Decoder);

  Inst.addOperand(MCOperand::createImm(Imm));

  return MCDisassembler::Success;
}
