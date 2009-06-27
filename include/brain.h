#ifndef BRAIN_H
#define BRAIN_H

#include <vector>

#include "common.h"

class Brain{
public:
    static Brain* getBrain(int problem, int scenarioNumber);
    virtual PortMapping step(const PortMapping& output) = 0;
    bool finished(const PortMapping& output) const;

    virtual std::vector<pointF> getShipsPositions() const = 0;
    virtual int getShipsNumber() const = 0;

protected:
    Brain(int sn):scenarioNumber(sn),timestep(0){}
    int scenarioNumber;
    int timestep;
};

#endif //BRAIN_H
