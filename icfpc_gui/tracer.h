#ifndef TRACER_H
#define TRACER_H

#include <string>
#include <fstream>
#include <vector>

#include "common.h"

class Tracer{
public:
    Tracer(const std::string& file, int scenario_number);
    void add(const PortMapping& data, int timestep);
    void dump(int timestep);

private:
    std::ofstream of;
    PortMapping prev;
    std::vector<uint32> trace_data;
    void push_double(double value);
};

#endif //TRACER_H
