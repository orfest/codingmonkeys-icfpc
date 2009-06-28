#ifndef BRAIN3_H
#define BRAIN3_H

#include "brain.h"

#include "common.h"
#include "vector.h"

class VM;

class B3 : public Brain{
public:
    B3(int scenarioNumber, VM* vm);
    virtual PortMapping _step(const PortMapping& output);

    virtual std::vector<pointF> getShipsPositions(const PortMapping& output) const;
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
	Vector rToMaxTargEarth;
	Vector myAphelion, myPerihelion;
	Vector targAphelion, targPerihelion;
	enum State {measuring, 
				waitingJumpFrom, 
				jumpedFromCircular, 
				jumpedToCircular, 
				waitingJumpTo, 
				countering,
				following,
				steering,
			   } state;

	double transferTime;
	bool clockwise;
	bool targClockwise;
	int steeringSteps;

	void hohmannTransfer(PortMapping & actuators, double fromR, double toR, 
						bool toCircular, Vector curMeEarth) const;
	void estimateOrbit(const Vector & velocity, const Vector & position, 
						Vector & aphelionPos, Vector & perihelionPos) const;
	double getPolarAngle(const Vector& v) const;
	Vector getVectorFromPolarAngle(double angle) const;
};

#endif //BRAIN3_H
