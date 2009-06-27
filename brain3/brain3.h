#ifndef BRAIN3_H
#define BRAIN3_H

#include "brain.h"

#include "common.h"

class B3 : public Brain{
public:
    B3(int scenarioNumber);
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

#endif //BRAIN3_H
