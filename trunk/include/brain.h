#ifndef BRAIN_H
#define BRAIN_H

#include <vector>

#include "common.h"

class Brain{
public:
    static Brain* getBrain(int problem);
    virtual std::vector<PortValue> initialStep() = 0;
    virtual std::vector<PortValue> step(const std::vector<PortValue>& output) = 0;
    virtual bool finished() const = 0;
protected:
    Brain(){}
};

#endif //BRAIN_H
