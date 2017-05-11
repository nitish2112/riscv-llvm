//===-- RISCVAnalyzeImmediate.cpp - Analyze Immediates --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#include "RISCVAnalyzeImmediate.h"
#include "RISCV.h"
#include "llvm/Support/MathExtras.h"

using namespace llvm;

RISCVAnalyzeImmediate::Inst::Inst(unsigned O, unsigned I) : Opc(O), ImmOpnd(I) {}

// Add I to the instruction sequences.
void RISCVAnalyzeImmediate::AddInstr(InstSeqLs &SeqLs, const Inst &I) {
  // Add an instruction seqeunce consisting of just I.
  if (SeqLs.empty()) {
    SeqLs.push_back(InstSeq(1, I));
    return;
  }

  for (InstSeqLs::iterator Iter = SeqLs.begin(); Iter != SeqLs.end(); ++Iter)
    Iter->push_back(I);
}

void RISCVAnalyzeImmediate::GetInstSeqLsADDI(uint64_t Imm, unsigned RemSize,
                                             InstSeqLs &SeqLs) {
  GetInstSeqLs((Imm + 0x800ULL) & 0xfffffffffffff000ULL, RemSize, SeqLs);
  AddInstr(SeqLs, Inst(ADDI, Imm & 0xfffULL));
}

void RISCVAnalyzeImmediate::GetInstSeqLsORI(uint64_t Imm, unsigned RemSize,
                                           InstSeqLs &SeqLs) {
  GetInstSeqLs(Imm & 0xfffffffffffff000ULL, RemSize, SeqLs);
  AddInstr(SeqLs, Inst(ORI, Imm & 0xfffULL));
}

void RISCVAnalyzeImmediate::GetInstSeqLsSLL(uint64_t Imm, unsigned RemSize,
                                           InstSeqLs &SeqLs) {
  unsigned Shamt = countTrailingZeros(Imm);
  GetInstSeqLs(Imm >> Shamt, RemSize - Shamt, SeqLs);
  AddInstr(SeqLs, Inst(SLL, Shamt));
}

void RISCVAnalyzeImmediate::GetInstSeqLs(uint64_t Imm, unsigned RemSize,
                                        InstSeqLs &SeqLs) {
  uint64_t MaskedImm = Imm & (0xffffffffffffffffULL >> (64 - Size));

  // Do nothing if Imm is 0.
  if (!MaskedImm)
    return;

  // A single ADDI will do if RemSize <= 12.
  if (RemSize <= 12) {
    AddInstr(SeqLs, Inst(ADDI, MaskedImm));
    return;
  }

  // Shift if the lower 11-bit is cleared.
  if (!(Imm & 0xfff)) {
    GetInstSeqLsSLL(Imm, RemSize, SeqLs);
    return;
  }

  GetInstSeqLsADDI(Imm, RemSize, SeqLs);

  // If bit 12 is cleared, it doesn't make a difference whether the last
  // instruction is an ADDI or ORI. In that case, do not call GetInstSeqLsORI.
  if (Imm & 0x800) {
    InstSeqLs SeqLsORI;
    GetInstSeqLsORI(Imm, RemSize, SeqLsORI);
    SeqLs.append(std::make_move_iterator(SeqLsORI.begin()),
                 std::make_move_iterator(SeqLsORI.end()));
  }
}

// Replace a ADDI & SLL pair with a LUI.
// e.g. the following two instructions
//  ADDI 0x0111
//  SLL 18
// are replaced with
//  LUI 0x444
void RISCVAnalyzeImmediate::ReplaceADDISLLWithLUI(InstSeq &Seq) {
  // Check if the first two instructions are ADDI and SLL and the shift amount
  // is at least 12.
  if ((Seq.size() < 2) || (Seq[0].Opc != ADDI) ||
      (Seq[1].Opc != SLL) || (Seq[1].ImmOpnd < 12))
    return;

  // Sign-extend and shift operand of ADDI and see if it still fits in 12-bit.
  int64_t Imm = SignExtend64<12>(Seq[0].ImmOpnd);
  int64_t ShiftedImm = (uint64_t)Imm << (Seq[1].ImmOpnd - 12);

  if (!isInt<20>(ShiftedImm))
    return;

  // Replace the first instruction and erase the second.
  Seq[0].Opc = LUI;
  Seq[0].ImmOpnd = (unsigned)(ShiftedImm & 0xfffff);
  Seq.erase(Seq.begin() + 1);
}

void RISCVAnalyzeImmediate::GetShortestSeq(InstSeqLs &SeqLs, InstSeq &Insts) {
  InstSeqLs::iterator ShortestSeq = SeqLs.end();
  // The length of an instruction sequence is at most 7.
  unsigned ShortestLength = 8;

  for (InstSeqLs::iterator S = SeqLs.begin(); S != SeqLs.end(); ++S) {
    ReplaceADDISLLWithLUI(*S);
    assert(S->size() <= 7);

    if (S->size() < ShortestLength) {
      ShortestSeq = S;
      ShortestLength = S->size();
    }
  }

  Insts.clear();
  Insts.append(ShortestSeq->begin(), ShortestSeq->end());
}

const RISCVAnalyzeImmediate::InstSeq
&RISCVAnalyzeImmediate::Analyze(uint64_t Imm, unsigned Size,
                                bool LastInstrIsADDI) {
  this->Size = Size;

  if (Size == 32) {
    ADDI = RISCV::ADDI;
    ORI = RISCV::ORI;
    SLL = RISCV::SLL;
    LUI = RISCV::LUI;
  } else {
    ADDI = RISCV::ADDI64;
    ORI = RISCV::ORI64;
    SLL = RISCV::SLL64;
    LUI = RISCV::LUI64;
  }

  InstSeqLs SeqLs;

  // Get the list of instruction sequences.
  if (LastInstrIsADDI | !Imm)
    GetInstSeqLsADDI(Imm, Size, SeqLs);
  else
    GetInstSeqLs(Imm, Size, SeqLs);

  // Set Insts to the shortest instruction sequence.
  GetShortestSeq(SeqLs, Insts);

  return Insts;
}
