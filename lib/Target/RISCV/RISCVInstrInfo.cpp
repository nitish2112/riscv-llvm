//===-- RISCVInstrInfo.cpp - RISCV Instruction Information ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the RISCV implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "RISCVInstrInfo.h"
#include "RISCV.h"
#include "RISCVSubtarget.h"
#include "RISCVTargetMachine.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_CTOR_DTOR
#include "RISCVGenInstrInfo.inc"

using namespace llvm;

RISCVInstrInfo::RISCVInstrInfo(RISCVSubtarget &sti)
  : RISCVGenInstrInfo(RISCV::ADJCALLSTACKDOWN, RISCV::ADJCALLSTACKUP),
    RI(sti), STI(sti) {}

void RISCVInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator Position,
                                 const DebugLoc &DL,
                                 unsigned DestinationRegister,
                                 unsigned SourceRegister,
                                 bool KillSource) const {
  if (STI.isRV64()) {
    if (RISCV::GPRRegClass.contains(DestinationRegister)) {
      BuildMI(MBB, Position, DL, get(RISCV::ADDIW), DestinationRegister)
        .addReg(SourceRegister, getKillRegState(KillSource))
        .addImm(0);
    } else {
      BuildMI(MBB, Position, DL, get(RISCV::ADDI64), DestinationRegister)
        .addReg(SourceRegister, getKillRegState(KillSource))
        .addImm(0);
    }
  } else {
    BuildMI(MBB, Position, DL, get(RISCV::ADDI), DestinationRegister)
      .addReg(SourceRegister, getKillRegState(KillSource))
      .addImm(0);
  }
}

void RISCVInstrInfo::getLoadStoreOpcodes(const TargetRegisterClass *RC,
                                         unsigned &LoadOpcode,
                                         unsigned &StoreOpcode) const {
  if (RC == &RISCV::GPRRegClass){
    LoadOpcode = RISCV::LW;
    StoreOpcode = RISCV::SW;
  } else if (RC == &RISCV::GPR64RegClass) {
    LoadOpcode = RISCV::LD;
    StoreOpcode = RISCV::SD;
  } else
    llvm_unreachable("Unsupported regclass to load or store");
}


void RISCVInstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         unsigned SrcReg, bool IsKill, int FI,
                                         const TargetRegisterClass *RC,
                                         const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (I != MBB.end())
    DL = I->getDebugLoc();

  unsigned LoadOpcode, StoreOpcode;
  getLoadStoreOpcodes(RC, LoadOpcode, StoreOpcode);

  BuildMI(MBB, I, DL, get(StoreOpcode))
      .addReg(SrcReg, getKillRegState(IsKill))
      .addFrameIndex(FI)
      .addImm(0);
}

void RISCVInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator I,
                                          unsigned DestReg, int FI,
                                          const TargetRegisterClass *RC,
                                          const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (I != MBB.end())
    DL = I->getDebugLoc();

  unsigned LoadOpcode, StoreOpcode;
  getLoadStoreOpcodes(RC, LoadOpcode, StoreOpcode);

  BuildMI(MBB, I, DL, get(LoadOpcode), DestReg).addFrameIndex(FI).addImm(0);
}

/// Adjust SP by Amount bytes.
void RISCVInstrInfo::adjustStackPtr(unsigned SP, int64_t Amount,
                                    MachineBasicBlock &MBB,
                                    MachineBasicBlock::iterator I) const {
  DebugLoc DL;
  if (Amount == 0)
    return;

  if (isInt<12>(Amount)) {
    // addi sp, sp, amount
    BuildMI(MBB, I, DL, get(RISCV::ADDI), SP).
      addReg(SP).addImm(Amount);
  } else {
    basePlusImmediate (SP, SP, Amount, MBB, I, DL);
  }
}

void RISCVInstrInfo::loadImmediate(unsigned ScratchReg,
                                   int64_t Imm, MachineBasicBlock &MBB,
                                   MachineBasicBlock::iterator II,
                                   const DebugLoc &DL) const {
  MachineRegisterInfo &RegInfo = MBB.getParent()->getRegInfo();
  const TargetRegisterClass *RC = STI.isRV64() ?
    &RISCV::GPR64RegClass : &RISCV::GPRRegClass;

  if (Imm == 0)
    return;

  int64_t LuiImm = ((Imm + 0x800) >> 12) & 0xfffff;

  if ((LuiImm != 0) && (SignExtend64<12> (Imm) == 0)) {
    BuildMI(MBB, II, DL, get(RISCV::LUI), ScratchReg)
      .addImm(LuiImm);
  } else if ((LuiImm == 0) && (SignExtend64<12> (Imm) != 0)) {
    BuildMI(MBB, II, DL, get(RISCV::ADDI), ScratchReg)
      .addReg(RISCV::X0_32)
      .addImm(SignExtend64<12> (Imm));
  } else {
    // Create TempReg here because Virtual register expect as SSA form.
    // So ADDI ScratchReg, ScratchReg, Imm is not allow.
    unsigned TempReg = RegInfo.createVirtualRegister(RC);

    BuildMI(MBB, II, DL, get(RISCV::LUI), TempReg)
      .addImm(LuiImm);

    BuildMI(MBB, II, DL, get(RISCV::ADDI), ScratchReg)
      .addReg(TempReg, getKillRegState (true))
      .addImm(SignExtend64<12> (Imm));
  }
}

unsigned
RISCVInstrInfo::basePlusImmediate(unsigned DstReg, unsigned BaseReg,
                                  int64_t Imm, MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator II,
                                  const DebugLoc &DL) const {
  MachineRegisterInfo &RegInfo = MBB.getParent()->getRegInfo();
  const TargetRegisterClass *RC = STI.isRV64() ?
    &RISCV::GPR64RegClass : &RISCV::GPRRegClass;
  unsigned ScratchReg = RegInfo.createVirtualRegister(RC);

  if (DstReg == 0)
    DstReg = ScratchReg;

  loadImmediate (ScratchReg, Imm, MBB, II, DL);
  BuildMI(MBB, II, DL, get(RISCV::ADD), DstReg)
    .addReg(ScratchReg)
    .addReg(BaseReg);

  return ScratchReg;
}

unsigned
RISCVInstrInfo::basePlusImmediateStripOffset(unsigned BaseReg, int64_t &Imm,
                                             MachineBasicBlock &MBB,
                                             MachineBasicBlock::iterator II,
                                             const DebugLoc &DL) const {
  MachineRegisterInfo &RegInfo = MBB.getParent()->getRegInfo();
  const TargetRegisterClass *RC = STI.isRV64() ?
    &RISCV::GPR64RegClass : &RISCV::GPRRegClass;
  unsigned ScratchReg1 = RegInfo.createVirtualRegister(RC);
  unsigned ScratchReg2 = RegInfo.createVirtualRegister(RC);

  int64_t LuiImm = ((Imm + 0x800) >> 12) & 0xfffff;

  BuildMI(MBB, II, DL, get(RISCV::LUI), ScratchReg1)
    .addImm(LuiImm);

  BuildMI(MBB, II, DL, get(RISCV::ADD), ScratchReg2)
    .addReg(ScratchReg1, getKillRegState (true))
    .addReg(BaseReg);

  Imm = SignExtend64<12> (Imm);

  return ScratchReg2;
}

//===----------------------------------------------------------------------===//
// Branch Analysis
//===----------------------------------------------------------------------===//

static bool isUncondBranch(unsigned Opcode) {
  if (Opcode == RISCV::PseudoBR ||
      Opcode == RISCV::PseudoBR64)
    return true;
  return false;
}

static bool isIndirectBranch(unsigned Opcode) {
  if (Opcode == RISCV::PseudoBRIND ||
      Opcode == RISCV::PseudoBRIND64)
    return true;
  return false;
}

static bool isCondBranch(unsigned Opcode) {

  if (Opcode == RISCV::BEQ ||
      Opcode == RISCV::BNE ||
      Opcode == RISCV::BLT ||
      Opcode == RISCV::BGE ||
      Opcode == RISCV::BLTU ||
      Opcode == RISCV::BGEU ||
      Opcode == RISCV::BEQ64 ||
      Opcode == RISCV::BNE64 ||
      Opcode == RISCV::BLT64 ||
      Opcode == RISCV::BGE64 ||
      Opcode == RISCV::BLTU64 ||
      Opcode == RISCV::BGEU64)
    return true;
  return false;
}

void RISCVInstrInfo::BuildCondBr(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                                 const DebugLoc &DL,
                                 ArrayRef<MachineOperand> Cond) const {
  unsigned Opc = Cond[0].getImm();
  const MCInstrDesc &MCID = get(Opc);
  MachineInstrBuilder MIB = BuildMI(&MBB, DL, MCID);

  for (unsigned I = 1; I < Cond.size(); ++I) {
    if (Cond[I].isReg())
      MIB.addReg(Cond[I].getReg());
    else if (Cond[I].isImm())
      MIB.addImm(Cond[I].getImm());
    else
       assert(false && "Cannot copy operand");
  }
  MIB.addMBB(TBB);
}

unsigned RISCVInstrInfo::insertBranch(MachineBasicBlock &MBB,
                                      MachineBasicBlock *TBB,
                                      MachineBasicBlock *FBB,
                                      ArrayRef<MachineOperand> Cond,
                                      const DebugLoc &DL,
                                      int *BytesAdded) const {
  // Shouldn't be a fall through.
  assert(TBB && "InsertBranch must not be told to insert a fallthrough");

  // Two-way Conditional branch.
  if (FBB) {
    BuildCondBr(MBB, TBB, DL, Cond);
    BuildMI(&MBB, DL, get(RISCV::PseudoBR)).addMBB(FBB);
    return 2;
  }

  // One way branch.
  // Unconditional branch.
  if (Cond.empty())
    BuildMI(&MBB, DL, get(RISCV::PseudoBR)).addMBB(TBB);
  else // Conditional branch.
    BuildCondBr(MBB, TBB, DL, Cond);
  return 1;
}

// removeBranch will remove last branch or last two
// conditional and unconditional branches which let BranchFolding
// could do the right thing.
unsigned RISCVInstrInfo::removeBranch(MachineBasicBlock &MBB,
                                      int *BytesRemoved) const {
  assert(!BytesRemoved && "code size not handled");

  MachineBasicBlock::iterator I = MBB.getLastNonDebugInstr();
  if (I == MBB.end())
    return 0;

  if (!isUncondBranch(I->getOpcode()) &&
      !isCondBranch(I->getOpcode()))
    return 0;

  // Remove the branch.
  I->eraseFromParent();

  I = MBB.end();

  if (I == MBB.begin()) return 1;
  --I;
  if (!isCondBranch(I->getOpcode()))
    return 1;

  // Remove the branch.
  I->eraseFromParent();
  return 2;
}

void RISCVInstrInfo::AnalyzeCondBr(const MachineInstr *Inst, unsigned Opc,
                                   MachineBasicBlock *&BB,
                                   SmallVectorImpl<MachineOperand> &Cond) const {
  int NumOp = Inst->getNumExplicitOperands();

  // for both int and fp branches, the last explicit operand is the
  // MBB.
  BB = Inst->getOperand(NumOp-1).getMBB();
  Cond.push_back(MachineOperand::CreateImm(Opc));

  for (int i=0; i<NumOp-1; i++)
    Cond.push_back(Inst->getOperand(i));
}

bool RISCVInstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                   MachineBasicBlock *&TBB,
                                   MachineBasicBlock *&FBB,
                                   SmallVectorImpl<MachineOperand> &Cond,
                                   bool AllowModify) const {
  // Start from the bottom of the block and work up, examining the
  // terminator instructions.
  MachineBasicBlock::iterator I = MBB.end();
  while (I != MBB.begin()) {
    --I;
    if (I->isDebugValue())
      continue;

    // Working from the bottom, when we see a non-terminator
    // instruction, we're done.
    if (!isUnpredicatedTerminator(*I))
      break;

    // A terminator that isn't a branch can't easily be handled
    // by this analysis.
    if (!I->isBranch())
      return true;

    // Cannot handle indirect branches.
    if (isIndirectBranch (I->getOpcode()))
      return true;

    // Handle unconditional branches.
    if (isUncondBranch (I->getOpcode())) {
      if (!AllowModify) {
        TBB = I->getOperand(0).getMBB();
        continue;
      }

      // If the block has any instructions after a J, delete them.
      while (std::next(I) != MBB.end())
        std::next(I)->eraseFromParent();
      Cond.clear();
      FBB = nullptr;

      // Delete the J if it's equivalent to a fall-through.
      if (MBB.isLayoutSuccessor(I->getOperand(0).getMBB())) {
        TBB = nullptr;
        I->eraseFromParent();
        I = MBB.end();
        continue;
      }

      // TBB is used to indicate the unconditinal destination.
      TBB = I->getOperand(0).getMBB();
      continue;
    }

    // Handle conditional branches.
    if (isCondBranch (I->getOpcode())) {
      // Working from the bottom, handle the first conditional branch.
      if (!Cond.empty())
        return true;

      FBB = TBB;
      MachineInstr *LastInst = &*I;
      AnalyzeCondBr(LastInst, I->getOpcode(), TBB, Cond);
      return false;
    }

    return true;
  }

  return false;
}
