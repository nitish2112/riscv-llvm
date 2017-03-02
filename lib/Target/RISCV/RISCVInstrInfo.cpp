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

RISCVInstrInfo::RISCVInstrInfo() : RISCVGenInstrInfo(RISCV::ADJCALLSTACKDOWN, RISCV::ADJCALLSTACKUP) {}

void RISCVInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator Position,
                                 const DebugLoc &DL,
                                 unsigned DestinationRegister,
                                 unsigned SourceRegister,
                                 bool KillSource) const {
  if (!RISCV::GPRRegClass.contains(DestinationRegister, SourceRegister)) {
    llvm_unreachable("Impossible reg-to-reg copy");
  }

  BuildMI(MBB, Position, DL, get(RISCV::ADDI), DestinationRegister)
      .addReg(SourceRegister, getKillRegState(KillSource))
      .addImm(0);
}

void RISCVInstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         unsigned SrcReg, bool IsKill, int FI,
                                         const TargetRegisterClass *RC,
                                         const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (I != MBB.end())
    DL = I->getDebugLoc();

  if (RC == &RISCV::GPRRegClass)
    BuildMI(MBB, I, DL, get(RISCV::SW))
        .addReg(SrcReg, getKillRegState(IsKill))
        .addFrameIndex(FI)
        .addImm(0);
  else
    llvm_unreachable("Can't store this register to stack slot");
}

void RISCVInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator I,
                                          unsigned DestReg, int FI,
                                          const TargetRegisterClass *RC,
                                          const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (I != MBB.end())
    DL = I->getDebugLoc();

  if (RC == &RISCV::GPRRegClass)
    BuildMI(MBB, I, DL, get(RISCV::LW), DestReg).addFrameIndex(FI).addImm(0);
  else
    llvm_unreachable("Can't load this register from stack slot");
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
    unsigned Reg = loadImmediate (SP, Amount, MBB, I, DL);
    BuildMI(MBB, I, DL, get(RISCV::ADD), SP)
      .addReg(SP).addReg(Reg);
  }
}

unsigned RISCVInstrInfo::loadImmediate(unsigned BaseReg, int64_t Imm,
                                       MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator II,
                                       const DebugLoc &DL) const {
  MachineFunction &MF = *MBB.getParent();
  MachineRegisterInfo &RegInfo = MBB.getParent()->getRegInfo();
  const RISCVSubtarget &STI = MF.getSubtarget<RISCVSubtarget>();
  const TargetRegisterClass *RC = STI.is64Bit() ?
    &RISCV::GPR64RegClass : &RISCV::GPRRegClass;

  unsigned ScratchReg = RegInfo.createVirtualRegister(RC);

  int64_t LuiImm = ((Imm + 0x800) >> 12) & 0xfffff;
  if (LuiImm != 0) {
    BuildMI(MBB, II, DL, get(RISCV::LUI), ScratchReg)
      .addImm(LuiImm);
  }
  BuildMI(MBB, II, DL, get(RISCV::ADDI), ScratchReg).addReg(ScratchReg)
    .addImm(SignExtend64<12> (Imm));

  BuildMI(MBB, II, DL, get(RISCV::ADD), ScratchReg).addReg(ScratchReg)
    .addReg(BaseReg);

  return ScratchReg;
}


//===----------------------------------------------------------------------===//
// Branch Analysis
//===----------------------------------------------------------------------===//

static bool isUncondBranch(unsigned Opcode) {

  if (Opcode == RISCV::PseudoBR)
    return true;
  return false;
}

static bool isCondBranch(unsigned Opcode) {

  if (Opcode == RISCV::BEQ ||
      Opcode == RISCV::BNE ||
      Opcode == RISCV::BLT ||
      Opcode == RISCV::BGE ||
      Opcode == RISCV::BLTU ||
      Opcode == RISCV::BGEU)
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
