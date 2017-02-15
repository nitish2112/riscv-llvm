//===-- RISCVISelLowering.cpp - RISCV DAG Lowering Implementation  --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that RISCV uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#include "RISCVISelLowering.h"
#include "RISCV.h"
#include "RISCVRegisterInfo.h"
#include "RISCVSubtarget.h"
#include "RISCVTargetMachine.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/CodeGen/ValueTypes.h"
#include "llvm/IR/DiagnosticInfo.h"
#include "llvm/IR/DiagnosticPrinter.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "riscv-lower"

RISCVTargetLowering::RISCVTargetLowering(const TargetMachine &TM,
                                         const RISCVSubtarget &STI)
    : TargetLowering(TM), Subtarget(&STI) {

  // Set up the register classes.
  addRegisterClass(MVT::i32, &RISCV::GPRRegClass);

  // Compute derived properties from the register classes
  computeRegisterProperties(STI.getRegisterInfo());

  setStackPointerRegisterToSaveRestore(RISCV::X2_32);

  // TODO: add all necessary setOperationAction calls

  setOperationAction(ISD::BR_CC, MVT::i32, Expand);
  setOperationAction(ISD::SELECT_CC, MVT::i32, Custom);
  setBooleanContents(ZeroOrOneBooleanContent);

  setOperationAction(ISD::GlobalAddress, MVT::i32, Custom);

  // Function alignments (log2)
  setMinFunctionAlignment(3);
  setPrefFunctionAlignment(3);

  // inline memcpy() for kernel to see explicit copy
  MaxStoresPerMemset = MaxStoresPerMemsetOptSize = 128;
  MaxStoresPerMemcpy = MaxStoresPerMemcpyOptSize = 128;
  MaxStoresPerMemmove = MaxStoresPerMemmoveOptSize = 128;
}

SDValue RISCVTargetLowering::LowerOperation(SDValue Op, SelectionDAG &DAG) const {
  switch (Op.getOpcode()) {
  case ISD::GlobalAddress:
    return lowerGlobalAddress(Op, DAG);
  case ISD::SELECT_CC:
    return lowerSELECT_CC(Op, DAG);
  default:
    report_fatal_error("unimplemented operand");
  }
}

SDValue RISCVTargetLowering::lowerGlobalAddress(SDValue Op,
                                                SelectionDAG &DAG) const {
  SDLoc DL(Op);
  EVT Ty = Op.getValueType();
  GlobalAddressSDNode *N = cast<GlobalAddressSDNode>(Op);
  const GlobalValue *GV = N->getGlobal();
  int64_t Offset = N->getOffset();

  if (!isPositionIndependent() && !Subtarget->is64Bit()) {
    SDValue GAHi =
        DAG.getTargetGlobalAddress(GV, DL, Ty, Offset, RISCVII::MO_HI);
    SDValue GALo =
        DAG.getTargetGlobalAddress(GV, DL, Ty, Offset, RISCVII::MO_LO);
    SDValue MNHi = SDValue(DAG.getMachineNode(RISCV::LUI, DL, Ty, GAHi), 0);
    SDValue MNLo =
        SDValue(DAG.getMachineNode(RISCV::ADDI, DL, Ty, MNHi, GALo), 0);
    return MNLo;
  } else {
    llvm_unreachable("Unable to lowerGlobalAddress");
  }
}

SDValue RISCVTargetLowering::lowerSELECT_CC(SDValue Op, SelectionDAG &DAG) const {
  SDValue LHS = Op.getOperand(0);
  SDValue RHS = Op.getOperand(1);
  SDValue TrueV = Op.getOperand(2);
  SDValue FalseV = Op.getOperand(3);
  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(4))->get();
  SDLoc DL(Op);

  switch (CC) {
  default:
    break;
  case ISD::SETGT:
  case ISD::SETLE:
  case ISD::SETUGT:
  case ISD::SETULE:
    CC = ISD::getSetCCSwappedOperands(CC);
    std::swap(LHS, RHS);
    break;
  }

  SDValue TargetCC = DAG.getConstant(CC, DL, MVT::i32);

  SDVTList VTs = DAG.getVTList(Op.getValueType(), MVT::Glue);
  SDValue Ops[] = {LHS, RHS, TargetCC, TrueV, FalseV};

  return DAG.getNode(RISCVISD::SELECT_CC, DL, VTs, Ops);
}

MachineBasicBlock *
RISCVTargetLowering::EmitInstrWithCustomInserter(MachineInstr &MI,
                                                 MachineBasicBlock *BB) const {
  const TargetInstrInfo &TII = *BB->getParent()->getSubtarget().getInstrInfo();
  DebugLoc DL = MI.getDebugLoc();

  assert(MI.getOpcode() == RISCV::Select && "Unexpected instr type to insert");

  // To "insert" a SELECT instruction, we actually have to insert the diamond
  // control-flow pattern.  The incoming instruction knows the destination vreg
  // to set, the condition code register to branch on, the true/false values to
  // select between, and a branch opcode to use.
  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  MachineFunction::iterator I = ++BB->getIterator();

  // ThisMBB:
  // ...
  //  TrueVal = ...
  //  jmp_XX r1, r2 goto Copy1MBB
  //  fallthrough --> Copy0MBB
  MachineBasicBlock *ThisMBB = BB;
  MachineFunction *F = BB->getParent();
  MachineBasicBlock *Copy0MBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *Copy1MBB = F->CreateMachineBasicBlock(LLVM_BB);

  F->insert(I, Copy0MBB);
  F->insert(I, Copy1MBB);
  // Update machine-CFG edges by transferring all successors of the current
  // block to the new block which will contain the Phi node for the select.
  Copy1MBB->splice(Copy1MBB->begin(), BB,
                   std::next(MachineBasicBlock::iterator(MI)), BB->end());
  Copy1MBB->transferSuccessorsAndUpdatePHIs(BB);
  // Next, add the true and fallthrough blocks as its successors.
  BB->addSuccessor(Copy0MBB);
  BB->addSuccessor(Copy1MBB);

  // Insert Branch if Flag
  unsigned LHS = MI.getOperand(1).getReg();
  unsigned RHS = MI.getOperand(2).getReg();
  int CC = MI.getOperand(3).getImm();
  unsigned Opcode = -1;
  switch (CC) {
  case ISD::SETEQ:
    Opcode = RISCV::BEQ;
    break;
  case ISD::SETNE:
    Opcode = RISCV::BNE;
    break;
  case ISD::SETLT:
    Opcode = RISCV::BLT;
    break;
  case ISD::SETGE:
    Opcode = RISCV::BGE;
    break;
  case ISD::SETULT:
    Opcode = RISCV::BLTU;
    break;
  case ISD::SETUGE:
    Opcode = RISCV::BGEU;
    break;
  default:
    report_fatal_error("unimplemented select CondCode " + Twine(CC));
  }

  BuildMI(BB, DL, TII.get(Opcode))
    .addReg(LHS)
    .addReg(RHS)
    .addMBB(Copy1MBB);

  // Copy0MBB:
  //  %FalseValue = ...
  //  # fallthrough to Copy1MBB
  BB = Copy0MBB;

  // Update machine-CFG edges
  BB->addSuccessor(Copy1MBB);

  // Copy1MBB:
  //  %Result = phi [ %FalseValue, Copy0MBB ], [ %TrueValue, ThisMBB ]
  // ...
  BB = Copy1MBB;
  BuildMI(*BB, BB->begin(), DL, TII.get(RISCV::PHI), MI.getOperand(0).getReg())
      .addReg(MI.getOperand(5).getReg())
      .addMBB(Copy0MBB)
      .addReg(MI.getOperand(4).getReg())
      .addMBB(ThisMBB);

  MI.eraseFromParent(); // The pseudo instruction is gone now.
  return BB;
}

// Calling Convention Implementation
#include "RISCVGenCallingConv.inc"

// Transform physical registers into virtual registers
SDValue RISCVTargetLowering::LowerFormalArguments(
    SDValue Chain, CallingConv::ID CallConv, bool IsVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &DL,
    SelectionDAG &DAG, SmallVectorImpl<SDValue> &InVals) const {

  switch (CallConv) {
  default:
    llvm_unreachable("Unsupported calling convention");
  case CallingConv::C:
    break;
  }

  MachineFunction &MF = DAG.getMachineFunction();
  MachineRegisterInfo &RegInfo = MF.getRegInfo();

  if (IsVarArg) {
    llvm_unreachable("VarArg not supported");
  }

  // Assign locations to all of the incoming arguments.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, IsVarArg, MF, ArgLocs, *DAG.getContext());
  CCInfo.AnalyzeFormalArguments(Ins, CC_RISCV32);

  for (auto &VA : ArgLocs) {
    if (VA.isRegLoc()) {
      // Arguments passed in registers
      EVT RegVT = VA.getLocVT();
      switch (RegVT.getSimpleVT().SimpleTy) {
      case MVT::i32: {
        const unsigned VReg =
            RegInfo.createVirtualRegister(&RISCV::GPRRegClass);
        RegInfo.addLiveIn(VA.getLocReg(), VReg);
        SDValue ArgIn = DAG.getCopyFromReg(Chain, DL, VReg, RegVT);

        InVals.push_back(ArgIn);
        break;
      }
      default:
        DEBUG(dbgs() << "LowerFormalArguments Unhandled argument type: "
                     << RegVT.getEVTString() << "\n");
        llvm_unreachable("unhandled argument type");
      }
    } else {
      llvm_unreachable("Defined with too many args");
    }
  }
  return Chain;
}

// Lower a call to a callseq_start + CALL + callseq_end chain, and add input 
// and output parameter nodes.
SDValue
RISCVTargetLowering::LowerCall(CallLoweringInfo &CLI,
                                 SmallVectorImpl<SDValue> &InVals) const {
  SelectionDAG &DAG = CLI.DAG;
  SDLoc &DL = CLI.DL;
  SmallVectorImpl<ISD::OutputArg> &Outs = CLI.Outs;
  SmallVectorImpl<SDValue> &OutVals = CLI.OutVals;
  SmallVectorImpl<ISD::InputArg> &Ins = CLI.Ins;
  SDValue Chain = CLI.Chain;
  SDValue Callee = CLI.Callee;
  CLI.IsTailCall = false;
  CallingConv::ID CallConv = CLI.CallConv;
  bool IsVarArg = CLI.IsVarArg;
  EVT PtrVT = getPointerTy(DAG.getDataLayout());

  if (IsVarArg) {
    llvm_unreachable("LowerCall with varargs not implemented");
  }

  MachineFunction &MF = DAG.getMachineFunction();

  // Analyze the operands of the call, assigning locations to each operand.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState ArgCCInfo(CallConv, IsVarArg, MF, ArgLocs, *DAG.getContext());
  ArgCCInfo.AnalyzeCallOperands(Outs, CC_RISCV32);

  // Get a count of how many bytes are to be pushed on the stack.
  unsigned NumBytes = ArgCCInfo.getNextStackOffset();

  Chain = DAG.getCALLSEQ_START(Chain,
                                 DAG.getIntPtrConstant(NumBytes, DL, true),
                                 DL);

  // Copy argument values to their designated locations.
  SmallVector<std::pair<unsigned, SDValue>, 8> RegsToPass;
  SDValue StackPtr;
  for (unsigned I = 0, E = ArgLocs.size(); I != E; ++I) {
    CCValAssign &VA = ArgLocs[I];
    SDValue ArgValue = OutVals[I];

    // Promote the value if needed.
    // For now, only handle fully promoted arguments.
    switch (VA.getLocInfo()) {
    case CCValAssign::Full:
      break;
    default:
      llvm_unreachable("Unknown loc info!");
    }

    if (VA.isRegLoc())
      // Queue up the argument copies and emit them at the end.
      RegsToPass.push_back(std::make_pair(VA.getLocReg(), ArgValue));
    else {
      assert(VA.isMemLoc() && "Argument not register or memory");
      llvm_unreachable("Passing arguments via the stack not yet implemented");
    }
  }

  SDValue Glue;

  // Build a sequence of copy-to-reg nodes, chained and glued together.
  for (unsigned I = 0, E = RegsToPass.size(); I != E; ++I) {
    Chain = DAG.getCopyToReg(Chain, DL, RegsToPass[I].first,
                             RegsToPass[I].second, Glue);
    Glue = Chain.getValue(1);
  }

  if (isa<GlobalAddressSDNode>(Callee)) {
    Callee = lowerGlobalAddress(Callee, DAG);
  } else if (isa<ExternalSymbolSDNode>(Callee)) {
    llvm_unreachable(
        "lowerExternalSymbol, needed for lowerCall, not yet handled");
  }

  // The first call operand is the chain and the second is the target address.
  SmallVector<SDValue, 8> Ops;
  Ops.push_back(Chain);
  Ops.push_back(Callee);

  // Add argument registers to the end of the list so that they are
  // known live into the call.
  for (unsigned I = 0, E = RegsToPass.size(); I != E; ++I)
    Ops.push_back(DAG.getRegister(RegsToPass[I].first,
                                  RegsToPass[I].second.getValueType()));

  // Add a register mask operand representing the call-preserved registers.
  const TargetRegisterInfo *TRI = Subtarget->getRegisterInfo();
  const uint32_t *Mask = TRI->getCallPreservedMask(MF, CallConv);
  assert(Mask && "Missing call preserved mask for calling convention");
  Ops.push_back(DAG.getRegisterMask(Mask));

  // Glue the call to the argument copies, if any.
  if (Glue.getNode())
    Ops.push_back(Glue);

  // Emit the call.
  SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);
  Chain = DAG.getNode(RISCVISD::CALL, DL, NodeTys, Ops);
  Glue = Chain.getValue(1);

  // Mark the end of the call, which is glued to the call itself.
  Chain = DAG.getCALLSEQ_END(Chain,
                             DAG.getConstant(NumBytes, DL, PtrVT, true),
                             DAG.getConstant(0, DL, PtrVT, true),
                             Glue, DL);
  Glue = Chain.getValue(1);

  // Assign locations to each value returned by this call.
  SmallVector<CCValAssign, 16> RetLocs;
  CCState RetCCInfo(CallConv, IsVarArg, MF, RetLocs, *DAG.getContext());
  RetCCInfo.AnalyzeCallResult(Ins, RetCC_RISCV32);

  // Copy all of the result registers out of their specified physreg.
  for (unsigned I = 0, E = RetLocs.size(); I != E; ++I) {
    CCValAssign &VA = RetLocs[I];

    // Copy the value out, gluing the copy to the end of the call sequence.
    SDValue RetValue = DAG.getCopyFromReg(Chain, DL, VA.getLocReg(),
                                          VA.getLocVT(), Glue);
    Chain = RetValue.getValue(1);
    Glue = RetValue.getValue(2);

    InVals.push_back(Chain.getValue(0));
  }

  return Chain;
}

SDValue
RISCVTargetLowering::LowerReturn(SDValue Chain, CallingConv::ID CallConv,
                               bool IsVarArg,
                               const SmallVectorImpl<ISD::OutputArg> &Outs,
                               const SmallVectorImpl<SDValue> &OutVals,
                               const SDLoc &DL, SelectionDAG &DAG) const {
  if (IsVarArg) {
    report_fatal_error("VarArg not supported");
  }

  // Stores the assignment of the return value to a location
  SmallVector<CCValAssign, 16> RVLocs;

  // Info about the registers and stack slot
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(), RVLocs,
                 *DAG.getContext());

  CCInfo.AnalyzeReturn(Outs, RetCC_RISCV32);

  SDValue Flag;
  SmallVector<SDValue, 4> RetOps(1, Chain);

  // Copy the result values into the output registers.
  for (unsigned i = 0, e = RVLocs.size(); i < e; ++i) {
    CCValAssign &VA = RVLocs[i];
    assert(VA.isRegLoc() && "Can only return in registers!");

    Chain = DAG.getCopyToReg(Chain, DL, VA.getLocReg(), OutVals[i], Flag);

    // Guarantee that all emitted copies are stuck together
    Flag = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(VA.getLocReg(), VA.getLocVT()));
  }

  RetOps[0] = Chain; // Update chain.

  // Add the flag if we have it.
  if (Flag.getNode()) {
    RetOps.push_back(Flag);
  }

  return DAG.getNode(RISCVISD::RET_FLAG, DL, MVT::Other, RetOps);
}

const char *RISCVTargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch ((RISCVISD::NodeType)Opcode) {
  case RISCVISD::FIRST_NUMBER:
    break;
  case RISCVISD::RET_FLAG:
    return "RISCVISD::RET_FLAG";
  case RISCVISD::CALL:
    return "RISCVISD::CALL";
  case RISCVISD::SELECT_CC:
    return "RISCVISD::SELECT_CC";
  }
  return nullptr;
}
