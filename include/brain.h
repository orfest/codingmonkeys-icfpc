#ifndef BRAIN_H
#define BRAIN_H

#include <vector>
#include <queue>
#include "vector.h"
#include "operations.h"
#include "common.h"

class VM;

class Brain{
public:
    static Brain* getBrain(int problem, int scenarioNumber, VM* vm);
    PortMapping step(const PortMapping& output);
	virtual PortMapping _step(const PortMapping& output) = 0;
    bool finished(const PortMapping& output) const;

    std::vector<pointF> getShipsPositions() const{
        return getShipsPositions(prevInput);
    }
    virtual std::vector<pointF> getShipsPositions(const PortMapping& output) const = 0;
    virtual int getShipsNumber() const = 0;

	static PortMapping prevResult;
	static PortMapping prevInput;

protected:
    static const double fuelEps;
    Brain(int sn, VM* vm_);
    int scenarioNumber;
    int timestep;
	
	
	std::queue<Operation*> operation_list;
	double time_shift; 
	STATE state;

    VM* vm;
    std::vector<Orbit> orbits;
    void simulateAndGetOrbits();
    bool isClockwise(const Vector& newPosition, const Vector& prevPosition) const;
    double getPolarAngle(const Vector& v) const;

	PortMapping & fuelOveruseFailsafe(const PortMapping & sensors, PortMapping & actuators);
    int estimateTimeToPoint(const Vector& point) const;
    int estimateTimeToPerihelion(const Orbit& orbit) const{
        return estimateTimeToPoint(orbit.maxR);
    }
    int estimateTimeToAphelion(const Orbit& orbit) const{
        return estimateTimeToPoint(orbit.minR);
    }

	void estimateOrbit(const Vector & velocity, const Vector & position, 
						Vector & aphelionPos, Vector & perihelionPos) const;
	Vector getVectorFromPolarAngle(double angle, double scale = 1.0) const;
	double getPhaseDifference(const Vector & a, const Vector & b) const;
	bool isPhaseWithinEpsCircleAware(const Vector & vec, const Vector & aphOrPer, 
								  const Vector & perOrAph, double epsilon = 0.001) const;
};
#endif //BRAIN_H
