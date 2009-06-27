#ifndef BRAIN1_H
#define BRAIN1_H

#include "brain.h"

#include "common.h"

class B1 : public Brain{
public:
    B1(int scenarioNumber);
    virtual PortMapping step(const PortMapping& output);

    virtual std::vector<pointF> getShipsPositions() const;
    virtual int getShipsNumber() const;

private:
    PortMapping prevResult;
    PortMapping prevInput;
};

#endif //BRAIN1_H
