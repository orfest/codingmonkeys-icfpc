#ifndef VM_H
#define VM_H

#include <string>
#include <vector>

#include "common.h"

class VM{
public:
    VM(const std::string& file);
    std::vector<PortValue> step(const std::vector<PortValue>& input);
};

#endif //VM_H
