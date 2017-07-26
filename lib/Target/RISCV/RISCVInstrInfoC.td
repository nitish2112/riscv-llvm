//===- RISCVInstrInfoC.td - Compressed RISCV instructions -*- tblgen-*-----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

class Stack_Load<bits<3> funct3, string OpcodeStr, SDPatternOperator Op,
                 RegisterClass cls, DAGOperand opnd> :
      CI<funct3, 0b10, (outs cls:$rd), (ins opnd:$imm),
         OpcodeStr#"\t$rd, $imm", []> {
  let Inst{11-7} = rd;
  let mayLoad = 1;
}

def CLWSP  : Stack_Load<0b010, "c.lwsp", load, GPR, addr_sp_imm6u_word>,
             Requires<[HasC]>;
