#ifndef OPERATIONS_H
#define OPERATIONS_H

#include <vector>
#include <queue>
#include "vector.h"

#include "common.h"

struct Orbit{
    bool clockwise;
	Vector minR;
	Vector maxR;
};


enum STATE{
	WAITING,
	RUNNING,
	COMPLETE
};

class Operation{
public:
    Operation():timestep(0),clockwise(false),state(WAITING),target(){}
	virtual PortMapping step(const PortMapping& output) = 0;
	int timestep;
	bool clockwise;
	STATE state;
	Orbit target;
	void SetTarget(Orbit newTarget){ target = newTarget; };
};

class Hohman : public Operation {
public:
	virtual PortMapping step(const PortMapping& output);

	double transferTime;
	double r1;
	double r2;
};

class CircleToElliptic : public Operation {
public:
	virtual PortMapping step(const PortMapping& output);
	double GetTransferTime();
	double transferTime;
	double r1;
	double r2;
};

class CircleToEllipticFast : public Operation {
public:
	virtual PortMapping step(const PortMapping& output);
	double GetTransferTime();
	double transferTime;
	double r1;
	double r2;
};

class EllipticToCircle : public Operation {
public:
	virtual PortMapping step(const PortMapping& output);

	double transferTime;
	double r1;
	double r2;
};

class FreeFly : public Operation {
public:
	virtual PortMapping step(const PortMapping& output);

	double transferTime;
};

class FreeFlyToOpositPoint : public Operation {
public:
	virtual PortMapping step(const PortMapping& output);
	
	double transferTime;
};

class FreeFlyToCertainPoint : public Operation {
public:
	virtual PortMapping step(const PortMapping& output);
	
	double transferTime;
    double curDistance;
    double prevDistance;
    Vector point;
};

class FlipDirection : public Operation {
public:
	virtual PortMapping step(const PortMapping& output);
private:
    Vector prev;
};

class Accelerate : public Operation {
public:
	virtual PortMapping step(const PortMapping& output);
    void setDelta(double d) { delta = d; }
private:
    double delta;
    Vector prev;
};

class MeetShip : public Operation {
public:
    MeetShip():Operation(),searchSuccessful(false),ship(-1){}
	virtual PortMapping step(const PortMapping& output);
    void setShip(int d) { ship = d; }
private:
    int ship;
    Vector prev;
    bool searchSuccessful;
};

double estimateTimeToPerihelionFormula(const Vector& point, const Orbit& orbit);



#endif //BRAIN_H
