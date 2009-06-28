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
    Brain(int sn, VM* vm_);
    int scenarioNumber;
    int timestep;
	
	
	std::queue<Operation*> operation_list;
	double time_shift; 
	STATE state;

    VM* vm;
    std::vector<Orbit> orbits;
    void simulateAndGetOrbits();

	PortMapping & fuelOveruseFailsafe(const PortMapping & sensors, PortMapping & actuators);
};
#endif //BRAIN_H
