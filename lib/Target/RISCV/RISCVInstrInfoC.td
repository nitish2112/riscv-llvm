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
      CI<funct3, 0b10, (outs cls:$rd), (ins opnd:$offset),
         OpcodeStr#"\t$rd, $offset", []> {
  let Inst{11-7} = rd;
  let mayLoad = 1;
}

def CLWSP  : Stack_Load<0b010, "c.lwsp", load, GPR, addr_sp_imm6u_word>,
             Requires<[HasC]> {
  let Inst{3-2} = offset{7-6};
  let Inst{6-4} = offset{4-2};
}

class Stack_Store<bits<3> funct3, string OpcodeStr, SDPatternOperator Op,
                  RegisterClass cls, DAGOperand opnd> :
      CSS<funct3, 0b10, (outs), (ins cls:$rs2, opnd:$offset),
          OpcodeStr#"\t$rs2, $offset", []> {
  let mayStore = 1;
}

def CSWSP  : Stack_Store<0b110, "c.swsp", store, GPR, addr_sp_imm6u_word>,
             Requires<[HasC]> {
  let Inst{8-7}  = offset{7-6};
  let Inst{12-9} = offset{5-2};
}

class Reg_Load<bits<3> funct3, string OpcodeStr, SDPatternOperator Op,
               RegisterClass cls, DAGOperand opnd> :
      CL<funct3, 0b00, (outs cls:$rd), (ins opnd:$addr),
         OpcodeStr#"\t$rd, $addr",
         [(set cls:$rd, (Op opnd:$addr))]> {
  let mayLoad = 1;
}

def CLW  : Reg_Load<0b010, "c.lw", load, GPRC, addr_reg_imm5u_word>,
           Requires<[HasC]>;
