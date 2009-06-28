#ifndef BRAIN_H
#define BRAIN_H

#include <vector>
#include <queue>
#include "vector.h"
#include "operations.h"
#include "common.h"

class Brain{
public:
    static Brain* getBrain(int problem, int scenarioNumber);
    PortMapping step(const PortMapping& output);
	virtual PortMapping _step(const PortMapping& output) = 0;
    bool finished(const PortMapping& output) const;

    virtual std::vector<pointF> getShipsPositions() const = 0;
    virtual int getShipsNumber() const = 0;

	static PortMapping prevResult;
	static PortMapping prevInput;	
protected:
    Brain(int sn):scenarioNumber(sn),timestep(0){}
    int scenarioNumber;
    int timestep;
	
	
	std::queue<Operation*> operation_list;
	double time_shift; 
	STATE state;

	PortMapping & fuelOveruseFailsafe(const PortMapping & sensors, PortMapping & actuators);
};
#endif //BRAIN_H
