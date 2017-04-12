//===-- RISCVFrameLowering.cpp - RISCV Frame Information ------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the RISCV implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "RISCVFrameLowering.h"
#include "RISCVSubtarget.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

using namespace llvm;

bool RISCVFrameLowering::hasFP(const MachineFunction &MF) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();

  return (MF.getTarget().Options.DisableFramePointerElim(MF) ||
          MF.getFrameInfo().hasVarSizedObjects() ||
          MFI.isFrameAddressTaken() ||
          TRI->needsStackRealignment(MF));
}

void RISCVFrameLowering::emitPrologue(MachineFunction &MF,
                                      MachineBasicBlock &MBB) const {
  unsigned SP = RISCV::X2_32;
  unsigned FP = RISCV::X8_32;
  unsigned ADDI = RISCV::ADDI;

  assert(&MF.front() == &MBB && "Shrink-wrapping not yet supported");
  MachineFrameInfo &MFI = MF.getFrameInfo();

  const RISCVInstrInfo &TII =
      *static_cast<const RISCVInstrInfo *>(MF.getSubtarget().getInstrInfo());
  const RISCVRegisterInfo &TRI =
      *static_cast<const RISCVRegisterInfo *>(MF.getSubtarget().getRegisterInfo());
  const RISCVSubtarget &STI = MF.getSubtarget<RISCVSubtarget>();
  const TargetRegisterClass *RC = STI.isRV64() ?
    &RISCV::GPR64RegClass : &RISCV::GPRRegClass;

  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc DL;

  // First, compute final stack size.
  uint64_t StackSize = MFI.getStackSize();

  // No need to allocate space on the stack.
  if (StackSize == 0 && !MFI.adjustsStack()) return;

  MachineModuleInfo &MMI = MF.getMMI();
  const MCRegisterInfo *MRI = MMI.getContext().getRegisterInfo();

  // Adjust stack.
  TII.adjustStackPtr(SP, -StackSize, MBB, MBBI);

  // emit ".cfi_def_cfa_offset StackSize"
  unsigned CFIIndex = MF.addFrameInst(
      MCCFIInstruction::createDefCfaOffset(nullptr, -StackSize));
  BuildMI(MBB, MBBI, DL, TII.get(TargetOpcode::CFI_INSTRUCTION))
      .addCFIIndex(CFIIndex);

  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();

  if (CSI.size()) {
    // Find the instruction past the last instruction that saves a callee-saved
    // register to the stack.
    for (unsigned i = 0; i < CSI.size(); ++i)
      ++MBBI;

    // Iterate over list of callee-saved registers and emit .cfi_offset
    // directives.
    for (std::vector<CalleeSavedInfo>::const_iterator I = CSI.begin(),
           E = CSI.end(); I != E; ++I) {
      int64_t Offset = MFI.getObjectOffset(I->getFrameIdx());
      unsigned Reg = I->getReg();
      unsigned CFIIndex = MF.addFrameInst(MCCFIInstruction::createOffset(
            nullptr, MRI->getDwarfRegNum(Reg, 1), Offset));
      BuildMI(MBB, MBBI, DL, TII.get(TargetOpcode::CFI_INSTRUCTION))
          .addCFIIndex(CFIIndex);
    }
  }

  // if framepointer enabled, set it to point to the stack pointer.
  if (hasFP(MF)) {
    if (isInt<12>(StackSize)) {
      BuildMI(MBB, MBBI, DL, TII.get(ADDI), FP).addReg(SP).addImm(StackSize)
        .setMIFlag(MachineInstr::FrameSetup);
    } else {
      TII.loadImmediate (FP, StackSize, MBB, MBBI, DL);

      BuildMI(MBB, MBBI, DL, TII.get(RISCV::ADD), FP).addReg(FP).addReg(SP)
        .setMIFlag(MachineInstr::FrameSetup);
    }
    // emit ".cfi_def_cfa_register $fp"
    unsigned CFIIndex = MF.addFrameInst(MCCFIInstruction::createDefCfaRegister(
        nullptr, MRI->getDwarfRegNum(FP, true)));
    BuildMI(MBB, MBBI, DL, TII.get(TargetOpcode::CFI_INSTRUCTION))
        .addCFIIndex(CFIIndex);

    // Realignment the stack for structure alignment
    // E.g. pass structure by stack and the structure need 16 byte alignment.
    // See the test case c-c++-common/torture/vector-shift2.c
    // myfunc2 function will return structure by stack.
    if (TRI.needsStackRealignment(MF)) {
      unsigned SP = RISCV::X2_32;
      unsigned ZERO = RISCV::X0_32;
      // ADDI $Reg, $zero, -MaxAlignment
      // AND  $sp, $sp, $Reg
      unsigned VR = MF.getRegInfo().createVirtualRegister(RC);
      assert(isInt<16>(MFI.getMaxAlignment()) &&
             "Function's alignment size requirement is not supported.");
      int MaxAlign = -(int)MFI.getMaxAlignment();

      BuildMI(MBB, MBBI, DL, TII.get(RISCV::ADDI), VR).addReg(ZERO) .addImm(MaxAlign);
      BuildMI(MBB, MBBI, DL, TII.get(RISCV::AND), SP).addReg(SP).addReg(VR);
    }
  }
}

void RISCVFrameLowering::emitEpilogue(MachineFunction &MF,
                                      MachineBasicBlock &MBB) const {
  unsigned SP = RISCV::X2_32;
  unsigned FP = RISCV::X8_32;
  unsigned ZERO = RISCV::X0_32;
  unsigned ADDI = RISCV::ADDI;

  MachineFrameInfo &MFI = MF.getFrameInfo();
  const RISCVInstrInfo &TII =
      *static_cast<const RISCVInstrInfo *>(MF.getSubtarget().getInstrInfo());
  const RISCVRegisterInfo &TRI =
      *static_cast<const RISCVRegisterInfo *>(MF.getSubtarget().getRegisterInfo());

  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  DebugLoc DL = MBBI->getDebugLoc();

  // Get the number of bytes from FrameInfo
  uint64_t StackSize = MFI.getStackSize();

  // Restore the stack pointer if framepointer enabled
  // and the stack have been realign or have variable length object
  if (hasFP(MF)) {
    // Find the first instruction that restores a callee-saved register.
    MachineBasicBlock::iterator I = MBBI;

    for (unsigned i = 0; i < MFI.getCalleeSavedInfo().size(); ++i)
      --I;

    // Insert instruction "addi $sp, $fp, 0" at this location.
    if (isInt<12>(-StackSize)) {
      BuildMI(MBB, I, DL, TII.get(ADDI), SP).addReg(FP).addImm(-StackSize);
    } else {
      TII.loadImmediate (SP, -StackSize, MBB, I, DL);
      BuildMI(MBB, I, DL, TII.get(RISCV::ADD), SP).addReg(FP).addReg(SP);
    }
  }

  if (!StackSize)
    return;
  else
    // Adjust stack.
    TII.adjustStackPtr(SP, StackSize, MBB, MBBI);
}

bool RISCVFrameLowering::
spillCalleeSavedRegisters(MachineBasicBlock &MBB,
                          MachineBasicBlock::iterator MI,
                          const std::vector<CalleeSavedInfo> &CSI,
                          const TargetRegisterInfo *TRI) const {
  if (CSI.empty())
    return false;

  MachineFunction &MF = *MBB.getParent();
  MachineBasicBlock *EntryBlock = &(&MF)->front();
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();

  for (unsigned i = 0, e = CSI.size(); i != e; ++i) {
    unsigned Reg = CSI[i].getReg();

    // Add the callee-saved register as live-in. It's killed at the spill.
    MBB.addLiveIn(Reg);

    // Insert the spill to the stack frame.
    bool IsKill = 1;
    const TargetRegisterClass *RC = TRI->getMinimalPhysRegClass(Reg);
    TII.storeRegToStackSlot(*EntryBlock, MI, Reg, IsKill,
                            CSI[i].getFrameIdx(), RC, TRI);
  }

  return true;
}

bool RISCVFrameLowering::
restoreCalleeSavedRegisters(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator MI,
                            const std::vector<CalleeSavedInfo> &CSI,
                            const TargetRegisterInfo *TRI) const {
  if (CSI.empty())
    return false;

  DebugLoc DL;
  if (MI != MBB.end()) DL = MI->getDebugLoc();

  MachineFunction &MF = *MBB.getParent();
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();

  for (unsigned i = 0, e = CSI.size(); i != e; ++i) {
    unsigned Reg = CSI[i].getReg();
    const TargetRegisterClass *RC = TRI->getMinimalPhysRegClass(Reg);
    TII.loadRegFromStackSlot(MBB, MI, Reg, CSI[i].getFrameIdx(), RC, TRI);
  }

  return true;
}

/// Mark \p Reg and all registers aliasing it in the bitset.
static void setAliasRegs(MachineFunction &MF, BitVector &SavedRegs,
                         unsigned Reg) {
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();
  for (MCRegAliasIterator AI(Reg, TRI, true); AI.isValid(); ++AI)
    SavedRegs.set(*AI);
}

void RISCVFrameLowering::determineCalleeSaves(MachineFunction &MF,
                                              BitVector &SavedRegs,
                                              RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);

  // Mark $fp as used if function has dedicated frame pointer.
  if (hasFP(MF))
    setAliasRegs(MF, SavedRegs, RISCV::X8_32);
}

int RISCVFrameLowering::getFrameIndexReference(const MachineFunction &MF,
                                               int FI,
                                               unsigned &FrameReg) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();

  FrameReg = hasFP(MF) ? RISCV::X8_32 : RISCV::X2_32;

  return MFI.getObjectOffset(FI) + MFI.getStackSize() -
         getOffsetOfLocalArea() + MFI.getOffsetAdjustment();
}

bool RISCVFrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  return !MF.getFrameInfo().hasVarSizedObjects();
}


