#ifndef BRAIN2_2_H
#define BRAIN2_2_H

#include "brain.h"

#include "common.h"
#include "vector.h"

class B2_2 : public Brain{
public:
    B2_2(int scenarioNumber);
    virtual PortMapping step(const PortMapping& output);

    virtual std::vector<pointF> getShipsPositions() const;
    virtual int getShipsNumber() const;

private:
    PortMapping prevResult;
    PortMapping prevInput;
	double me_r;
	double alp;
	double that_r;
	double transferTime;
	double transferTime2;
	double r1;
	double r2;
	double rt;
	bool clockwise;
	double secondparttime;
	bool me_clockwise;
    bool that_clockwise;
    bool needTurnAround;
    int startTime;
    double delta_v1;

    double getAngle(const Vector& v) const;
};

#endif //BRAIN2_H
