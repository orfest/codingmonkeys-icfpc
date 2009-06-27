#ifndef BRAIN2_H
#define BRAIN2_H

#include "brain.h"

#include "common.h"

class B2 : public Brain{
public:
    B2(int scenarioNumber);
    virtual PortMapping step(const PortMapping& output);

    virtual std::vector<pointF> getShipsPositions() const;
    virtual int getShipsNumber() const;

private:
    PortMapping prevResult;
    PortMapping prevInput;
	double r1;
	double r2;
	double transferTime;
	bool clockwise;
};

#endif //BRAIN2_H
