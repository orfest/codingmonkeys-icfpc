#ifndef OPERATIONS_H
#define OPERATIONS_H

#include <vector>
#include <queue>
#include "vector.h"

#include "common.h"

struct Orbit{
	Vector minR;
	Vector maxR;
};

enum OPERATION{
	INITIALIZATION,
	HOHMAN_ORBIT_TRANSFER,
	BI_ELLIPTIC_TRANSFER
};

enum STATE{
	WAITING,
	RUNNING,
	COMPLETE
};

class Operation{
public:
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




#endif //BRAIN_H
