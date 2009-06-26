#include "instr.h"

code_t Instr::getNoop(){
    return buildSInstr(NOOP, 0, 0);
}

code_t Instr::buildSInstr(opcode_t opcode, int imm, addr_t src){
    code_t op(0);
    op |= (opcode << 24);
    op |= (imm << 14);
    op |= src;
    return op;
}

opcode_t Instr::getBits(opcode_t op, int from, int to){
    op >>= from;
    op &= ((1 << (to-from+1))-1);
    return op;
}

opcode_t Instr::getSOpcode(code_t op){
    return getBits(op, 24, 27);
}

int Instr::getSImm(code_t op){
    return getBits(op, 14, 23);
}

addr_t Instr::getSReg(code_t op){
    return getBits(op, 0, 13);
}

opcode_t Instr::getCmpOpcode(code_t op){
    return getBits(op, 21, 23);
}

opcode_t Instr::getDOpcode(code_t op){
    return getBits(op, 28, 31);
}

addr_t Instr::getDReg1(code_t op){
    return getBits(op, 14, 27);
}

addr_t Instr::getDReg2(code_t op){
    return getBits(op, 0, 13);
}
