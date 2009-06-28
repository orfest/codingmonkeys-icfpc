#ifndef TRACER_H
#define TRACER_H

#include <string>
#include <fstream>
#include <vector>

#include "common.h"

struct Frame{
    int timestep;
    int count;
    PortMapping changes;

    Frame(const PortMapping& that, int step);
    std::ofstream& operator<<(std::ofstream& os) const;

    int zeropadding(int v) const;
};

std::ofstream& operator<<(std::ofstream& os, const Frame& frame);

class Tracer{
public:
    Tracer(const std::string& file, int scenario_number);
    void add(const PortMapping& data, int timestep);
    void dump(int timestep);

private:
    std::ofstream of;
    PortMapping prev;
    std::vector<uint32> trace_data;
    std::vector<Frame> frames;
    void push_double(double value);
};

#endif //TRACER_H
