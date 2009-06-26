#ifndef TRACER_H
#define TRACER_H

#include <string>
#include <vector>

#include "common.h"

class Tracer{
public:
    Tracer(const std::string& file);
    void add(const std::vector<PortValue>& data);
    void dump() const;
};

#endif //TRACER_H
