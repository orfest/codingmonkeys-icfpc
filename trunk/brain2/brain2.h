#ifndef BRAIN2_H
#define BRAIN2_H

#include "brain.h"

#include "common.h"
#include "vector.h"

class B2 : public Brain{
public:
    B2(int scenarioNumber, VM * vm);
    virtual PortMapping step(const PortMapping& output);

    virtual std::vector<pointF> getShipsPositions() const;
    virtual int getShipsNumber() const;

private:
    PortMapping prevResult;
    PortMapping prevInput;
	double me_r;
	double that_r;
	double transferTime;
	bool me_clockwise;
    bool that_clockwise;
    bool needTurnAround;
    int startTime;
    double delta_v1;

    double getAngle(const Vector& v) const;
};

#endif //BRAIN2_H
