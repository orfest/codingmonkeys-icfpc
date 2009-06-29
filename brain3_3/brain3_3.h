#ifndef BRAIN3_3_H
#define BRAIN3_3_H

#include "brain.h"

#include "common.h"
#include "vector.h"

class VM;
class B3_3 : public Brain{
public:
	B3_3(int scenarioNumber, VM* vm);
    virtual PortMapping _step(const PortMapping& output);

	virtual std::vector<pointF> getShipsPositions(const PortMapping& output) const;
    virtual int getShipsNumber() const;

private:
    PortMapping prevResult;
    PortMapping prevInput;
	int step;

	void getOrbits(const PortMapping & sensors);
};

#endif //BRAIN3_H
