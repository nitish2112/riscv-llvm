//===-- RISCVSubtarget.cpp - RISCV Subtarget Information ------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the RISCV specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "RISCV.h"
#include "RISCVFrameLowering.h"
#include "RISCVSubtarget.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "riscv-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "RISCVGenSubtargetInfo.inc"

void RISCVSubtarget::anchor() {}

RISCVSubtarget &RISCVSubtarget::initializeSubtargetDependencies(StringRef CPU,
                                                                StringRef FS) {
  std::string CPUName = CPU;
  if (CPUName.empty()){
    CPUName = "generic";
  }

  // Parse features string.
  ParseSubtargetFeatures(CPUName, FS);
  return *this;
}

RISCVSubtarget::RISCVSubtarget(const Triple &TT, const std::string &CPU,
                               const std::string &FS, const TargetMachine &TM)
    : RISCVGenSubtargetInfo(TT, CPU, FS), InstrInfo(), FrameLowering(*this),
      TLInfo(TM, *this), RISCVArchVersion(RV32), HasM(true), HasA(true){}
