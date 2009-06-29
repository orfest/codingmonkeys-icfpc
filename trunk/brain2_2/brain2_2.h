#ifndef BRAIN2_2_H
#define BRAIN2_2_H

#include "brain.h"

#include "common.h"
#include "vector.h"

class VM;

class B2_2 : public Brain{
public:
    B2_2(int scenarioNumber, VM* vm);
    virtual PortMapping _step(const PortMapping& output);

    virtual std::vector<pointF> getShipsPositions(const PortMapping& output) const;
    virtual int getShipsNumber() const;

private:
	int step;
	double alp;
	double r1;
	double r2;
	double rt;
	bool clockwise;

    double getAngle(const Vector& v) const;
};

#endif //BRAIN2_H
