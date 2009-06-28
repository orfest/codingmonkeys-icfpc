#ifndef BRAIN4_H
#define BRAIN4_H

#include "brain.h"

#include "common.h"

class B4 : public Brain{
public:
    B4(int scenarioNumber);
    virtual PortMapping _step(const PortMapping& output);

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

#endif //BRAIN4_H