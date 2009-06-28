#ifndef BRAIN4_H
#define BRAIN4_H

#include "brain.h"

#include "common.h"

class VM;

class B4 : public Brain{
public:
    B4(int scenarioNumber, VM* vm);
    virtual PortMapping _step(const PortMapping& output);

    virtual std::vector<pointF> getShipsPositions(const PortMapping& output) const;
    virtual int getShipsNumber() const;

private:
    PortMapping prevResult;
    PortMapping prevInput;
	double r1;
	double r2;
	double transferTime;
	bool clockwise;
};

#endif //BRAIN4_H