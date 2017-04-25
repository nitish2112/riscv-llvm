//===- RISCVInstrInfoA.td - Atomic RISCV instructions -*- tblgen-*---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//RV32
//TODO: add LR/SC and acq/rel

def AMOSWAP_W : InstA<"amoswap.w" , 0b0101111, 0b00000, 0b010, atomic_swap     , GPR, memreg>, Requires<[IsRV32, HasA]>;
def AMOADD_W  : InstA<"amoadd.w"  , 0b0101111, 0b00001, 0b010, atomic_load_add , GPR, memreg>, Requires<[IsRV32, HasA]>;
def AMOXOR_W  : InstA<"amoxor.w"  , 0b0101111, 0b00100, 0b010, atomic_load_xor , GPR, memreg>, Requires<[IsRV32, HasA]>;
def AMOAND_W  : InstA<"amoand.w"  , 0b0101111, 0b01100, 0b010, atomic_load_and , GPR, memreg>, Requires<[IsRV32, HasA]>;
def AMOOR_W   : InstA<"amoor.w"   , 0b0101111, 0b01000, 0b010, atomic_load_or  , GPR, memreg>, Requires<[IsRV32, HasA]>;
def AMOMIN_W  : InstA<"amomin.w"  , 0b0101111, 0b10000, 0b010, atomic_load_min , GPR, memreg>, Requires<[IsRV32, HasA]>;
def AMOMAX_W  : InstA<"amomax.w"  , 0b0101111, 0b10100, 0b010, atomic_load_max , GPR, memreg>, Requires<[IsRV32, HasA]>;
def AMOMINU_W : InstA<"amominu.w" , 0b0101111, 0b11000, 0b010, atomic_load_umin, GPR, memreg>, Requires<[IsRV32, HasA]>;
def AMOMAXU_W : InstA<"amomaxu.w" , 0b0101111, 0b11100, 0b010, atomic_load_umax, GPR, memreg>, Requires<[IsRV32, HasA]>;

def LR_W : InstLR<"lr.w", 0b010, GPR, memreg>, Requires<[HasA]>;
def SC_W : InstSC<"sc.w", 0b010, GPR, memreg>, Requires<[HasA]>;
