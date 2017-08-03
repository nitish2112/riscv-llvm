//===-- RISCVRegisterInfo.cpp - RISCV Register Information ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the RISCV implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "RISCVRegisterInfo.h"
#include "RISCV.h"
#include "RISCVSubtarget.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/IR/DebugInfo.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Target/TargetFrameLowering.h"
#include "llvm/Target/TargetInstrInfo.h"

#define DEBUG_TYPE "riscv-reg-info"

#define GET_REGINFO_TARGET_DESC
#include "RISCVGenRegisterInfo.inc"

using namespace llvm;

RISCVRegisterInfo::RISCVRegisterInfo(const RISCVSubtarget &STI)
      : RISCVGenRegisterInfo(RISCV::X1_32), Subtarget(STI) {}

const MCPhysReg *
RISCVRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  if(Subtarget.isRV64())
    return CSR_RV64_SaveList;
  else if(Subtarget.hasE())
    return CSR_RV32E_SaveList;
  else
    return CSR_RV32_SaveList;
}

BitVector RISCVRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());
  Reserved.set(RISCV::X0_64); // zero
  Reserved.set(RISCV::X0_32); // zero
  Reserved.set(RISCV::X2_64); // sp
  Reserved.set(RISCV::X2_32); // sp
  Reserved.set(RISCV::X3_64); // gp
  Reserved.set(RISCV::X3_32); // gp
  Reserved.set(RISCV::X4_64); // tp
  Reserved.set(RISCV::X4_32); // tp
  Reserved.set(RISCV::X8_64); // fp
  Reserved.set(RISCV::X8_32); // fp

  if (Subtarget.hasE()) {
    Reserved.set(RISCV::X16_32);
    Reserved.set(RISCV::X17_32);
    Reserved.set(RISCV::X18_32);
    Reserved.set(RISCV::X19_32);
    Reserved.set(RISCV::X20_32);
    Reserved.set(RISCV::X21_32);
    Reserved.set(RISCV::X22_32);
    Reserved.set(RISCV::X23_32);
    Reserved.set(RISCV::X24_32);
    Reserved.set(RISCV::X25_32);
    Reserved.set(RISCV::X26_32);
    Reserved.set(RISCV::X27_32);
    Reserved.set(RISCV::X28_32);
    Reserved.set(RISCV::X29_32);
    Reserved.set(RISCV::X30_32);
    Reserved.set(RISCV::X31_32);
  }
  // Reserve the base register if we need to both realign the stack and
  // allocate variable-sized objects at runtime. This should test the
  if (needsStackRealignment(MF) &&
      MF.getFrameInfo().hasVarSizedObjects()) {
    Reserved.set(RISCV::X9_64);
    Reserved.set(RISCV::X9_32);
  }

  return Reserved;
}

const uint32_t *RISCVRegisterInfo::getNoPreservedMask() const {
  return CSR_NoRegs_RegMask;
}

bool RISCVRegisterInfo::
requiresRegisterScavenging(const MachineFunction &MF) const {
  return true;
}

bool RISCVRegisterInfo::
trackLivenessAfterRegAlloc(const MachineFunction &MF) const {
  return true;
}

bool RISCVRegisterInfo::
requiresFrameIndexScavenging(const MachineFunction &MF) const {
  return true;
}

bool RISCVRegisterInfo::
requiresVirtualBaseRegisters(const MachineFunction &MF) const {
  return true;
}

// Return true if the instruction fit
// 16 bit load/store format.
static bool FitFrameBase16BitLoadStore (MachineInstr &MI, unsigned BaseReg,
                                        int64_t Offset) {
  unsigned Opc = MI.getOpcode();

  // BaseReg must be SP
  if (BaseReg != RISCV::X2_32 && BaseReg != RISCV::X2_64)
    return false;

  // Offset limitation is (imm6u << 2)
  if (!isUInt<6>(Offset))
    return false;
  if (Offset % 4 != 0)
    return false;

  if (Opc != RISCV::LW && Opc != RISCV::SW)
    return false;

  return true;
}

void RISCVRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                            int SPAdj, unsigned FIOperandNum,
                                            RegScavenger *RS) const {
  assert(SPAdj == 0 && "Unexpected");
  MachineInstr &MI = *II;
  MachineBasicBlock &MBB = *MI.getParent();
  MachineFunction &MF = *MBB.getParent();

  DEBUG(errs() << "\nFunction : " << MF.getName() << "\n";
        errs() << "<--------->\n" << MI);

  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  uint64_t stackSize = MF.getFrameInfo().getStackSize();
  int64_t Offset = MF.getFrameInfo().getObjectOffset(FrameIndex);

  DEBUG(errs() << "FrameIndex : " << FrameIndex << "\n"
               << "spOffset   : " << Offset << "\n"
               << "stackSize  : " << stackSize << "\n");

  unsigned SP = Subtarget.isRV64() ? RISCV::X2_64 : RISCV::X2_32;
  unsigned FP = Subtarget.isRV64() ? RISCV::X8_64 : RISCV::X8_32;
  unsigned BP = Subtarget.isRV64() ? RISCV::X9_64 : RISCV::X9_32;

  MachineFrameInfo &MFI = MF.getFrameInfo();
  const RISCVInstrInfo &TII =
       *static_cast<const RISCVInstrInfo *>(MF.getSubtarget().getInstrInfo());
  const RISCVRegisterInfo &RegInfo =
       *static_cast<const RISCVRegisterInfo *>(MF.getSubtarget().getRegisterInfo());
  const RISCVFrameLowering *TFI = getFrameLowering(MF);
  DebugLoc DL = MI.getDebugLoc();
  unsigned BasePtr = (TFI->hasFP(MF) ? FP : SP);

  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  int MinCSFI = 0;
  int MaxCSFI = -1;

  if (CSI.size()) {
    MinCSFI = CSI[0].getFrameIdx();
    MaxCSFI = CSI[CSI.size() - 1].getFrameIdx();
  }
  // Use SP as Base if sp_offset (offset + stackSize)
  // could fit in isInt<12>.
  // Then we could avoid expand new instruction for setting offset.
  if ((BasePtr == FP) &&
      !isInt<12>(Offset) &&
      isInt<12>(Offset + stackSize))
    BasePtr = SP;

  // When dynamically realigning the stack, use the frame pointer for
  // parameters, and the stack/base pointer for locals.
  // see the function foo in the test case gcc.dg/torture/pr70421.c
  if (RegInfo.needsStackRealignment(MF)) {
    assert(TFI->hasFP(MF) && "dynamic stack realignment without a FP!");
    bool isFixed = MFI.isFixedObjectIndex(FrameIndex);
    if (MFI.hasVarSizedObjects() && !MFI.isFixedObjectIndex(FrameIndex)) {
      BasePtr = BP;
    } else if (isFixed) {
      BasePtr = FP;
    } else {
      BasePtr = SP;
    }
  }

  // If the FrameIndex push a callee saved register, use SP as Base
  // To avoid FP not setting yet when -fno-omit-frame-pointer
  // E.g.
  //   addi    $sp, $sp, -56
  //   swi     $fp, [$sp + (44)] <= use SP as base
  //   addi    $fp, $sp, 0
  if (FrameIndex >= MinCSFI && FrameIndex <= MaxCSFI)
    BasePtr = SP;

  if (BasePtr == SP || BasePtr == BP)
    Offset += stackSize;

  // Fold imm into offset
  Offset += MI.getOperand(FIOperandNum + 1).getImm();

  // Transfer to 16 bit format
  // if the instruction fit 16 bit load/store limitation
  if (FitFrameBase16BitLoadStore (MI, BasePtr, Offset) && Subtarget.hasC()) {
    unsigned NewOpc;

    switch (MI.getOpcode()) {
    case RISCV::LW:
      NewOpc = RISCV::CLWSP;
      break;
    case RISCV::SW:
      NewOpc = RISCV::CSWSP;
      break;
    default:
      break;
      // should add llvm_unreach here later.
    }

    MI.setDesc(TII.get(NewOpc));
    MI.getOperand(FIOperandNum).ChangeToRegister(BasePtr, false);
    MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);
    return;
  // If the offset fits in an immediate, then directly encode it
  } else if (isInt<12>(Offset)) {
    MI.getOperand(FIOperandNum).ChangeToRegister(BasePtr, false);
    MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);
    return;
  } else {
    // Generate following code when Offset exceed 12-bit signed
    //   lui     ra, 1048544
    //   add     ra, ra, s0
    //   sw      a0, -176(ra)
    unsigned ScratchReg = TII.basePlusImmediateStripOffset(BasePtr, Offset,
                                                           MBB, II, DL);
    MI.getOperand(FIOperandNum).ChangeToRegister(ScratchReg, false, false, true);
    MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);
  }
}

unsigned RISCVRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const RISCVFrameLowering *TFI = getFrameLowering(MF);
  if(Subtarget.isRV64())
    return TFI->hasFP(MF) ? RISCV::X8_64 : RISCV::X2_64;
  else
    return TFI->hasFP(MF) ? RISCV::X8_32 : RISCV::X2_32;
}

const uint32_t *
RISCVRegisterInfo::getCallPreservedMask(const MachineFunction & /*MF*/,
                                        CallingConv::ID /*CC*/) const {
  if(Subtarget.isRV64())
    return CSR_RV64_RegMask;
  else if(Subtarget.hasE())
    return CSR_RV32E_RegMask;
  else
    return CSR_RV32_RegMask;
}

const TargetRegisterClass *
RISCVRegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                      unsigned Kind) const {
  if(Subtarget.isRV64())
    return &RISCV::GPR64RegClass;
  else
    return &RISCV::GPRRegClass;
}
