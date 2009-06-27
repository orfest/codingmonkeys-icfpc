#ifndef BRAIN_H
#define BRAIN_H

#include <vector>

#include "common.h"

class Brain{
public:
    static Brain* getBrain(int problem);
    virtual PortMapping initialStep() = 0;
    virtual PortMapping step(const PortMapping& output) = 0;
    virtual bool finished() const = 0;
    virtual std::vector<pointF> getShipsPositions() const = 0;
    virtual int getShipsNumber() const = 0;
protected:
    Brain(){}
};

#endif //BRAIN_H
