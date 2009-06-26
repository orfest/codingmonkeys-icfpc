#ifndef INSTR_H
#define INSTR_H

#include "common.h"

typedef code_t opcode_t;

class Instr{
public:
    static code_t getNoop();
    static code_t buildSInstr(opcode_t opcode, int imm, addr_t src);

    static opcode_t getSOpcode(code_t op);
    static int getSImm(code_t op);
    static addr_t getSReg(code_t op);
    static opcode_t getCmpOpcode(code_t op);

    static opcode_t getDOpcode(code_t op);
    static addr_t getDReg1(code_t op);
    static addr_t getDReg2(code_t op);

    static opcode_t getBits(opcode_t op, int from, int to);

    static const opcode_t S_TYPE_OP = 0;
// D-Type
    static const opcode_t ADD = 1;
    static const opcode_t SUB = 2;
    static const opcode_t MULT = 3;
    static const opcode_t DIV = 4;
    static const opcode_t OUTPUT = 5;
    static const opcode_t PHI = 6;

// S-Type
    static const opcode_t NOOP = 0;
    static const opcode_t CMPZ = 1;
    static const opcode_t SQRT = 2;
    static const opcode_t COPY = 3;
    static const opcode_t INPUT = 4;

// Conditionals
    static const opcode_t LTZ = 0;
    static const opcode_t LEZ = 1;
    static const opcode_t EQZ = 2;
    static const opcode_t GEZ = 3;
    static const opcode_t GTZ = 4;
};

#endif //INSTR_H
