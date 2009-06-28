#ifndef BRAIN1_H
#define BRAIN1_H

#include "brain.h"
#include "common.h"

class VM;

class B1 : public Brain{
public:
    B1(int scenarioNumber, VM* vm);
    virtual PortMapping _step(const PortMapping& output);

    virtual std::vector<pointF> getShipsPositions(const PortMapping& output) const;
    virtual int getShipsNumber() const;

private:
    
	int step;
};

#endif //BRAIN1_H
