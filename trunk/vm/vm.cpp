#include "vm.h"

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <exception>
#include <cmath>

#include "instr.h"

using namespace std;

bit_t bit(bool a){
    return a ? 1 : 0;
}

VM::VM(const string& file) : program_counter_register(0), status_register(0) {
    code_memory.resize(MEM_SIZE, Instr::getNoop());   
    data_memory.resize(MEM_SIZE, 0.0);
    ifstream is(file.c_str(), ios_base::binary | ios_base::in);
    if (is.fail()){
        cerr << "Failed to open program file " << file << "\n";
        throw exception("Failed to open program");
    }
    const int framesize = sizeof(data_t) + sizeof(code_t);
    is.seekg(0, ios::end);
    int length = is.tellg();
    if ((length % framesize) != 0){
        cerr << "Incorrect program file size " << length << "\n";
        throw exception("Incorrect program file size");
    }
    is.seekg(0, ios::beg);
    char buf[framesize];
    for (int cnt = 0; cnt < (length / framesize); cnt++){
        is.read(buf, framesize);
        code_t op;
        data_t data;
        if ((cnt & 1) == 1){
            op = *reinterpret_cast<code_t*>(buf);
            data = *reinterpret_cast<data_t*>(buf+sizeof(code_t));
        } else {
            op = *reinterpret_cast<code_t*>(buf+sizeof(data_t));
            data = *reinterpret_cast<data_t*>(buf);
        }
        data_memory[cnt] = data;
        code_memory[cnt] = op;
    }
}

PortMapping VM::step(const PortMapping& input){
    input_mapping = input;
    output_mapping.clear();
    for (program_counter_register = 0; program_counter_register < MEM_SIZE;){
        bool store_result = true;
        data_t res;

        code_t op = code_memory[program_counter_register];
        opcode_t opcode = Instr::getDOpcode(op);
        if (opcode == Instr::S_TYPE_OP){
            opcode = Instr::getSOpcode(op);
            int imm = Instr::getSImm(op);
            addr_t reg = Instr::getSReg(op);

            if (opcode == Instr::NOOP){
                res = data_memory[program_counter_register];
            } else if (opcode == Instr::CMPZ){
                store_result = false;
                data_t v = data_memory[program_counter_register];
                opcode_t cmp_opcode = Instr::getCmpOpcode(op);
                bit_t br;
                if (cmp_opcode == Instr::LTZ){
                    br = bit(v < 0);
                } else if (cmp_opcode == Instr::LEZ){
                    br = bit(v <= 0);
                } else if (cmp_opcode == Instr::EQZ){
                    br = bit(v == 0);
                } else if (cmp_opcode == Instr::GEZ){
                    br = bit(v >= 0);
                } else if (cmp_opcode == Instr::GTZ){
                    br = bit(v > 0);
                } else {
                    cerr << "Unknown CMPZ opcode!\n";
                    throw exception("Unknown CMPZ opcode!");
                }
                status_register = br;
            } else if (opcode == Instr::SQRT){
                if (data_memory[reg] < 0){
                    cerr << "Spec 1.5 says sqrt for negative values is undefined. Dying!\n";
                    throw exception("sqrt for negative values");
                }
                res = sqrt(data_memory[reg]);
            } else if (opcode == Instr::COPY){
                res = data_memory[reg];
            } else if (opcode == Instr::INPUT){
                res = do_input(reg);
            } else {
                cerr << "Unknown S-Type opcode!\n";
                throw exception("Unknown S-Type opcode!");
            }
        } else {
            addr_t reg1 = Instr::getDReg1(op);
            addr_t reg2 = Instr::getDReg2(op);

            if (opcode == Instr::ADD) {
                res = data_memory[reg1] + data_memory[reg2];
            } else if (opcode == Instr::SUB) {
                res = data_memory[reg1] - data_memory[reg2];
            } else if (opcode == Instr::MULT) {
                res = data_memory[reg1] * data_memory[reg2];
            } else if (opcode == Instr::DIV) {
                if (data_memory[reg2] == 0.0){
                    res = 0.0;
                } else {
                    res = data_memory[reg1] / data_memory[reg2];
                }
            } else if (opcode == Instr::OUTPUT) {
                store_result = false;
                do_output(reg1, data_memory[reg2]);
            } else if (opcode == Instr::PHI) {
                if (status_register == 1){
                    res = data_memory[reg1];
                } else {
                    res = data_memory[reg2];
                }
            } else {
                cerr << "Unknown D-Type opcode!\n";
                throw exception("Unknown D-Type opcode!");
            }
        }
        if (store_result){
            data_memory[program_counter_register] = res;
        }
        program_counter_register++;
    }
    return output_mapping;
}

data_t VM::do_input(addr_t reg){
    return input_mapping[reg];
}

void VM::do_output(addr_t reg, data_t value){
    output_mapping[reg] = value;
}
