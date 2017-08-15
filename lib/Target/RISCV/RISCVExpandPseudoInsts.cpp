//===-- RISCVExpandPseudoInsts.cpp - Expand pseudo instructions -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains a pass that expands pseudo instructions into target
// instructions to allow proper scheduling, if-conversion, and other late
// optimizations. This pass should be run after register allocation but before
// the post-regalloc scheduling pass.
//
//===----------------------------------------------------------------------===//

#include "RISCV.h"
#include "RISCVAnalyzeImmediate.h"
#include "RISCVInstrInfo.h"
#include "RISCVRegisterInfo.h"
#include "RISCVMachineFunctionInfo.h"
#include "RISCVSubtarget.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineInstrBundle.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h" // FIXME: for debug only. remove!
#include "llvm/Target/TargetFrameLowering.h"
#include "llvm/Target/TargetRegisterInfo.h"

using namespace llvm;

#define DEBUG_TYPE "riscv-pseudo"

static cl::opt<bool>
VerifyRISCVPseudo("verify-riscv-pseudo-expand", cl::Hidden,
                  cl::desc("Verify machine code after expanding RISCV pseudos"));

namespace {
  class RISCVExpandPseudo : public MachineFunctionPass {
  public:
    static char ID;
    RISCVExpandPseudo() : MachineFunctionPass(ID) {}

    const RISCVInstrInfo *TII;
    const TargetRegisterInfo *TRI;
    const RISCVSubtarget *STI;

    bool runOnMachineFunction(MachineFunction &Fn) override;

    MachineFunctionProperties getRequiredProperties() const override {
      return MachineFunctionProperties().set(
          MachineFunctionProperties::Property::NoVRegs);
    }

    StringRef getPassName() const override {
      return "RISCV pseudo instruction expansion pass";
    }

  private:
    void transferImpOps(MachineInstr &OldMI,
                        MachineInstrBuilder &UseMI, MachineInstrBuilder &DefMI);
    bool expandMI(MachineBasicBlock &MBB,
                  MachineBasicBlock::iterator MBBI,
                  MachineBasicBlock::iterator &NextMBBI);
    bool expandMBB(MachineBasicBlock &MBB);
    void expandMOV32BitImm(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator &MBBI);
    void expandMOV64BitImm(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator &MBBI);
  };
  char RISCVExpandPseudo::ID = 0;
}

/// transferImpOps - Transfer implicit operands on the pseudo instruction to
/// the instructions created from the expansion.
void RISCVExpandPseudo::transferImpOps(MachineInstr &OldMI,
                                       MachineInstrBuilder &UseMI,
                                       MachineInstrBuilder &DefMI) {
  const MCInstrDesc &Desc = OldMI.getDesc();
  for (unsigned i = Desc.getNumOperands(), e = OldMI.getNumOperands();
       i != e; ++i) {
    const MachineOperand &MO = OldMI.getOperand(i);
    assert(MO.isReg() && MO.getReg());
    if (MO.isUse())
      UseMI.add(MO);
    else
      DefMI.add(MO);
  }
}

void RISCVExpandPseudo::expandMOV32BitImm(MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator &MBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Opcode = MI.getOpcode();
  unsigned DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  const MachineOperand &MO = MI.getOperand(1);
  int32_t Imm = MO.getImm();
  int32_t Lo12 = SignExtend32<12>(Imm);
  int32_t Hi20 = ((Imm + 0x800) >> 12) & 0xfffff;
  MachineInstrBuilder LO12, HI20;

  unsigned LO12Opc = STI->isRV32() ? RISCV::ADDI : RISCV::ADDIW;
  unsigned HI20Opc = RISCV::LUI;

  if ((LO12Opc == RISCV::ADDIW) && STI->hasC() &&
      isIntN(6, Lo12))
    LO12Opc = RISCV::CADDIW;

  HI20 = BuildMI(MBB, MBBI, MI.getDebugLoc(), TII->get(HI20Opc))
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead));
  LO12 = BuildMI(MBB, MBBI, MI.getDebugLoc(), TII->get(LO12Opc), DstReg)
    .addReg(DstReg);

  switch (MO.getType()) {
  case MachineOperand::MO_Immediate: {
    LO12 = LO12.addImm(Lo12);
    HI20 = HI20.addImm(Hi20);
    break;
  }
  default:
    llvm_unreachable("expandMOV32BitImm operand not immediate value");
  }

  transferImpOps(MI, LO12, HI20);
  MI.eraseFromParent();
}

void RISCVExpandPseudo::expandMOV64BitImm(MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator &MBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Opcode = MI.getOpcode();
  unsigned DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  const MachineOperand &MO = MI.getOperand(1);
  RISCVAnalyzeImmediate AnalyzeImm;
  int64_t Imm = MO.getImm();
  MachineInstrBuilder MIB1, MIB2;

  const RISCVAnalyzeImmediate::InstSeq &Seq =
    AnalyzeImm.Analyze(Imm, 64, false);

  RISCVAnalyzeImmediate::InstSeq::const_iterator Inst = Seq.begin();

  if (Inst->Opc == RISCV::LUI64) {
    MIB1 = BuildMI(MBB, MBBI, MI.getDebugLoc(), TII->get(Inst->Opc))
           .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
           .addImm(Inst->ImmOpnd);
  }
  else {
    MIB1 = BuildMI(MBB, MBBI, MI.getDebugLoc(), TII->get(Inst->Opc))
           .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
           .addReg(RISCV::X0_64)
           .addImm(SignExtend64<12>(Inst->ImmOpnd));
  }

  // The remaining instructions in the sequence are handled here.
  for (++Inst; Inst != Seq.end(); ++Inst) {
    MIB2 = BuildMI(MBB, MBBI, MI.getDebugLoc(), TII->get(Inst->Opc))
           .addReg(DstReg)
           .addReg(DstReg)
           .addImm(SignExtend64<12>(Inst->ImmOpnd));
  }

  transferImpOps(MI, MIB2, MIB1);
  MI.eraseFromParent();
}

bool RISCVExpandPseudo::expandMI(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator MBBI,
                                 MachineBasicBlock::iterator &NextMBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Opcode = MI.getOpcode();
  switch (Opcode) {
    default:
      return false;
    case RISCV::MOVi32imm:
      expandMOV32BitImm(MBB, MBBI);
      return true;
    case RISCV::MOVi64imm:
      expandMOV64BitImm(MBB, MBBI);
      return true;
    }
}

bool RISCVExpandPseudo::expandMBB(MachineBasicBlock &MBB) {
  bool Modified = false;

  MachineBasicBlock::iterator MBBI = MBB.begin(), E = MBB.end();
  while (MBBI != E) {
    MachineBasicBlock::iterator NMBBI = std::next(MBBI);
    Modified |= expandMI(MBB, MBBI, NMBBI);
    MBBI = NMBBI;
  }

  return Modified;
}

bool RISCVExpandPseudo::runOnMachineFunction(MachineFunction &MF) {
  STI = &static_cast<const RISCVSubtarget &>(MF.getSubtarget());
  TII = STI->getInstrInfo();
  TRI = STI->getRegisterInfo();

  bool Modified = false;
  for (MachineFunction::iterator MFI = MF.begin(), E = MF.end(); MFI != E;
       ++MFI)
    Modified |= expandMBB(*MFI);
  if (VerifyRISCVPseudo)
    MF.verify(this, "After expanding RISCV pseudo instructions.");
  return Modified;
}

/// createRISCVExpandPseudoPass - returns an instance of the pseudo instruction
/// expansion pass.
FunctionPass *llvm::createRISCVExpandPseudoPass() {
  return new RISCVExpandPseudo();
}
