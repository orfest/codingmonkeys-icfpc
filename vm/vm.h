#ifndef VM_H
#define VM_H

#include <string>
#include <vector>

#include "common.h"

typedef char bit_t;

bit_t bit(bool a);

class VM{
public:
    VM(const std::string& file);
    PortMapping step(const PortMapping& input);
private:
    static const int NUM_ADDR_BITS = 14;
    static const int MEM_SIZE = (1 << NUM_ADDR_BITS);
    static const int MAX_ADDR = MEM_SIZE - 1;

    std::vector<code_t> code_memory;
    std::vector<data_t> data_memory;

//    addr_t program_counter_register; wiped out
    bit_t status_register;          //!!TODO!!: should it be initialized at every program run?

    data_t do_input(addr_t reg);
    void do_output(addr_t reg, data_t value);

    PortMapping input_mapping;
    PortMapping output_mapping;
};

#endif //VM_H
