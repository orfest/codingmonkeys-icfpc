#ifndef BRAIN4_A_H
#define BRAIN4_A_H

#include "brain.h"

#include "common.h"

class VM;

class B4_a : public Brain{
public:
    B4_a(int scenarioNumber, VM* vm);
    virtual PortMapping _step(const PortMapping& output);

    virtual std::vector<pointF> getShipsPositions(const PortMapping& output) const;
    virtual int getShipsNumber() const;

private:
	double r1;
	double r2;
	bool clockwise;
    bool start;
    int did;

	int step;
	double alp;
	double rt;

};

#endif //BRAIN4_H