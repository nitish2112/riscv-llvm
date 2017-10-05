#define DEBUG_TYPE "xloops-info"
#include <stdio.h>
#include <map>
#include "RISCV.h"
#include "RISCVTargetMachine.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Metadata.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineLoopInfo.h"                                     
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/TargetRegisterInfo.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/Support/raw_ostream.h"


// nitish: self defined macro for debugging
#define DPRINTF(...) printf(__VA_ARGS__); fflush(stdout);

// TBD: Add comments to this file later

using namespace llvm;

namespace {
  struct Xloops : public MachineFunctionPass {
    typedef MachineBasicBlock::instr_iterator InstrIter;
    typedef MachineBasicBlock::reverse_instr_iterator ReverseInstrIter;

    TargetMachine &TM;

    static char ID;
    Xloops(TargetMachine &tm)
      : MachineFunctionPass(ID), TM(tm) { }

    virtual char *getPassName() {
      return "RISCV Xloops Pass";
    }

    // function for every basic block in a machine function
    bool runOnMachineBasicBlock_xloops(MachineBasicBlock &MBB);

    void getAnalysisUsage(AnalysisUsage &AU) const override {
      AU.addRequired<MachineLoopInfo>(); //LoopInfoWrapperPass>(); 
      AU.addPreserved<MachineLoopInfo>();
      MachineFunctionPass::getAnalysisUsage(AU);  
    }

    // main run on machine function
    bool runOnMachineFunction(MachineFunction &F) {

      DPRINTF("Run on Machine Function for Xloops pass\n\n");
      bool Changed = false;
      for (MachineFunction::iterator FI = F.begin(), FE = F.end();
           FI != FE; ++FI) {
        Changed |= runOnMachineBasicBlock_xloops(*FI);

      }
    }
  };

  char Xloops::ID = 0;
} // end of anonymous namespace


// handle xloop hints
bool Xloops::runOnMachineBasicBlock_xloops(MachineBasicBlock &MBB) {

  DPRINTF("RISCVXloops:runOnMachineBasicBlock_xloops\n");
  bool Changed = false;
  const LLVMContext &Ctx = MBB.getParent()->getFunction()->getContext();

  // TBD: Look into more robust logic to identify the terminator
  // instruction and the induction variable increment using MachineLoopInfo
  // analysis

  /* To identify a tagged loop for transparent specialization, loop through
     all the instructions in the current basic block and if a hint is present,
     assume that the terminator instruction for the basic block is the BNE
     instruction we would like to modify to the corresponding loop
     instruction */

  // get the Loop Info
  MachineLoopInfo* MLI = &getAnalysis<MachineLoopInfo>();
  // Find the IR representation of this Basic Block.
  MachineLoop* ML = MLI->getLoopFor(&MBB);
  // Iterate through all instructions in the basic block
  MachineFunction *MF = MBB.getParent();
  const TargetInstrInfo *TII = MF->getSubtarget().getInstrInfo();

  bool isUnorderedFor = false;
  // llvm.loop.unordered_for is marked on the back edges of a loop. Therefore,
  // we iterate through each inst in the latch block and check
  // whether its metadata contains llvm.loop.unroll.enable.                                                                        
  if (const BasicBlock *BB = MBB.getBasicBlock()) {                       
    if (MDNode *LoopID =                                                     
            BB->getTerminator()->getMetadata(LLVMContext::MD_loop)) {    
      StringRef Name = "llvm.loop.unordered_for.enable";
      for (unsigned i = 1, e = LoopID->getNumOperands(); i < e; ++i) {
        MDNode *MD = dyn_cast<MDNode>(LoopID->getOperand(i));
          if (!MD)                                                                  
            continue;                                                               
                                                                            
          MDString *S = dyn_cast<MDString>(MD->getOperand(0));                      
          if (!S)                                                                   
            continue;                                                               
                                                 
          DPRINTF("MD String found is %s\n", Name);                          
          if (Name.equals(S->getString())) {                                          
            isUnorderedFor = true;          
            DPRINTF("unordered_for hint found\n"); 
          }                                                   
      }
    }                                                                        
  }                                                                          


//  for (InstrIter I = MBB.instr_begin(); I != MBB.instr_end(); ++I) {
    // Both IR and Machine instruction classes have DebugLoc data member. 
//    if ( I->getDebugLoc() && I->getDebugLoc().getScope() ) {
     if ( isUnorderedFor )  { 
      DPRINTF("Inside isUnorderedFor\n");
      // This machine basic block does belong to a Loop and this block is the latch block
      // Later: This will also have that if the loop is tagged.
      if (!ML) {
        DPRINTF("ML is null\n");
        return Changed;
      }
      if ( (ML->getBlocks()).empty() ) {
        DPRINTF("ML does not have basic blocks\n");
        return Changed;
      }
      if ( ML->getBottomBlock() != &MBB ) {
        DPRINTF("Bottom Block is not MBB\n");
        return Changed;
      }

      if ( ML && !(ML->getBlocks()).empty() &&  ML->getBottomBlock() == &MBB  ) { // ML->getLoopLatch() == MBB.getBasicBlock() ) { 
        MachineInstr* CondBr = &*(MBB.getFirstTerminator());
	DPRINTF("[Found]: hint for_u, inst is: %d RISCV::BNE is %d\n", CondBr->getOpcode(), RISCV::BNE);
        if ( CondBr->getOpcode() == RISCV::BNE64 || CondBr->getOpcode() == RISCV::BLT64 ) {
	  DPRINTF("[BNE]: detected, changing instruction to for_u\n");

          // Change the instruction to for.u
          CondBr->setDesc(TII->get(RISCV::FOR_U64));

          MachineInstrBuilder MIB;
          bool legal = false;

          // Insert Jump Instruction
          for( unsigned i = 0, e = CondBr->getDesc().getNumOperands(); i < e; ++i ) {
            MachineOperand &MO = CondBr->getOperand(i);
            if ( MO.isMBB() ) {
	      DPRINTF("[BNE]: found the target of the bne instruction\n");
              // TgMBB - MBB the branch is pointing to
              // the target for the branch instruction is present as a pointer
              // to that basic block in machine operand
              MachineBasicBlock* TgtMBB = MO.getMBB();
              // JmpMBB - MBB where the jump instruction is to be inserted
              MachineBasicBlock* JmpMBB = NULL;
              // iterate through the predecessor list and get the MBB
              // The predecessor of the TgtMBB can be either the latch block
              // or the loop header. So mark the loop header as JmpMBB
              // nitish: eg. 
              //    preheader
              //      |
              //     BB1  
              //    / |  
              //   /  BB2
              //  |   |
              //   \  BB3
              //    \ |
              //    latch  //taginfo exits in latch block
              //      |
              //    exit
              //
              // In such a case, MBB=latch, TgtMBB=BB1 and JmpMBB becomes preheader

              for ( MachineBasicBlock::pred_iterator b = TgtMBB->pred_begin(),
                    e = TgtMBB->pred_end(); b != e; ++b ) {
                if ( (*b)->getNumber() != MBB.getNumber() ) {
                  DPRINTF("[Jump]: found the BB where to insert the jump instruction\n");
                  JmpMBB = (*b);
                }
              }
              // nitish: start 
              // for some benchmarks, e.g. quicksort, the successor may has no
              // predecessor. The loop structure for quicksort benchmark looks
              // like:
              //    preheader
              //         \
              //     BB1  |
              //    / |  /
              //   /  BB2
              //  |   |
              //   \  BB3
              //    \ |
              //    latch  //taginfo exits in latch block
              //      |
              //    exit
              //
              // In such a case, MBB=latch, TgtMBB=BB1 and JmpMBB becomes NULL
              // Our tricky hack is to step into the next block of TgtMBB if
              // JmpBB is NULL. With the following code, JmpBB = preheader
              if (!JmpMBB) {
                MachineBasicBlock *TempMBB = *(TgtMBB->succ_begin());
                for ( MachineBasicBlock::pred_iterator b = TempMBB->pred_begin(),
                      e = TempMBB->pred_end(); b != e; ++b ) {
                  if ( (*b)->getNumber() != TgtMBB->getNumber() )
                    JmpMBB = (*b);
                }
              }
              // nitish: end

              if (JmpMBB) {
	 	DPRINTF("[Jump]: Insering jump instruction, size of JmpMBB is %d\n", JmpMBB->size());
                // InstrIter pos (*(JmpMBB->instr_end()));
                // InstrIter pos (JmpMBB->instr_end());
                //              BB    iterator  debug_loc    MCInstrDesc
                // MIB = BuildMI(*(JmpMBB), pos, DebugLoc(), TII->get(RISCV::JAL64));
                MIB = BuildMI(*(JmpMBB), JmpMBB->instr_end(), DebugLoc(), TII->get(RISCV::PseudoBR64));
                // MIB = BuildMI(*(JmpMBB), DebugLoc(), TII->get(RISCV::PseudoBR64));
                legal = true;
              }
            }  // MO.isMBB
          }  // loop to find the target of the branch inst
          // if (!legal) continue;
          if (!legal) return Changed;

          // Splice MBB
          // Create a new machine basic block by copying the IR basic block
          // to NewMBB
          DPRINTF("[Split]: Splitting the Basic Blocks\n");
          // Create a copy of the latch block
          MachineBasicBlock *NewMBB =  // 
          MF->CreateMachineBasicBlock(MBB.getBasicBlock());  //
          // Transfer the sucessors of latch block to this new BB
          NewMBB->transferSuccessors(&MBB);  //
          // Add edge from old latch block to this newMBB
          MBB.addSuccessor(NewMBB); //
          // Insert the New BB after the old latch block
          MF->insert(std::next((MachineFunction::iterator(&MBB)), 1), NewMBB);  // 
          // Find the iterator to the last instruction in the old latch block
          InstrIter split = MBB.instr_end();
          split--;
          // Take instruction at location split in MBB and insert it at 
          // NewMBB->begin(). i.e. insert for.u/bne instruction in NewMBB
          NewMBB->splice(NewMBB->begin(), &MBB, split );  //
          // if not set, the NewMBB gets optimized away
          NewMBB->setIsEHPad();  ////

          // Set jump target
          MIB.addMBB(NewMBB);    //// 
  
          // Set the Rd register in JAL instruction
          // ??? Leave it for now, may be by default it will be set to r0 ??? 

          Changed |= true;
          //break;

          //    preheader
          //         \  \
          //     BB1  |  \
          //    / |  /    \
          //   / BB2      |
          //  |   |       |
          //  |  BB3      |
          //  |   |      /
          //   \ latch  /  
          //    \ |    /  
          //    newMBB  
          //      |
          //    exit
          //
     
        } // if condBranch is bne or blt
      } // if bottom block is MBB
     }  // if unordered_for
   // }
 // }

  DPRINTF("Returning from RISCVXloops:runOnMachineBasicBlock_xloops\n");
  return Changed;
}

FunctionPass *llvm::createRISCVXloopsPass(RISCVTargetMachine &tm) {
  return new Xloops(tm);
}

