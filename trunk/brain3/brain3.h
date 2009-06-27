#ifndef BRAIN3_H
#define BRAIN3_H

#include "brain.h"

#include "common.h"
#include "vector.h"

class B3 : public Brain{
public:
    B3(int scenarioNumber);
    virtual PortMapping step(const PortMapping& output);

    virtual std::vector<pointF> getShipsPositions() const;
    virtual int getShipsNumber() const;

private:
    PortMapping prevResult;
    PortMapping prevInput;
	Vector startMeEarth;
	Vector startTargEarth;
	double rFromMin;
	double rFromMax;
	double rToMin;
	double rToMax;
	bool rFromKnown;
	bool rToKnown;
	double periodFrom;
	double periodTo;
	enum State {measuring, 
				waitingJumpFrom, 
				jumpedFromCircular, 
				jumpedToCircular, 
				waitingJumpTo, 
				following
			   } state;

	double transferTime;
	bool clockwise;
};

#endif //BRAIN3_H
