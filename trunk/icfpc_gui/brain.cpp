#include "brain.h"

#include <exception>
#include <iostream>
#include <assert.h>

#include "brain1.h"
#include "brain2.h"
#include "brain2_2.h"
#include "brain3.h"
#include "brain4.h"
#include "vector.h"

PortMapping Brain::prevInput = PortMapping();
PortMapping Brain::prevResult = PortMapping();

Brain* Brain::getBrain(int problem, int scenarioNumber){
    if (problem == 0) {
        return new B1(scenarioNumber);
	} else if (problem == 1) {
        return new B2_2(scenarioNumber);
    /*} else if (problem == 2) {
        return new B3(scenarioNumber);
    } else if (problem == 3) {
        return new B4(scenarioNumber);*/
    } else {
        throw new std::exception("Unknown problem type");
    }
}

bool Brain::finished(const PortMapping& output) const{
    data_t score = output.find(SCORE_PORT)->second;
    if (score == 0) return false;
    std::cout << 
        "Scenario "     << scenarioNumber << " completed!\n" << 
        "Final score: " << score << "\n" << 
        "Time: "        << timestep << "\n" << 
        "Fuel remaining: " << output.find(FUEL_PORT)->second << std::endl;
    return true;
}

PortMapping & Brain::fuelOveruseFailsafe(const PortMapping & sensors, PortMapping & actuators) {
	if (sensors.empty())
		return actuators;

	double fuelAvailable = sensors.find(FUEL_PORT)->second;
	if (fuelAvailable < 0.1)
		fuelAvailable = 0.0;
	Vector delta(actuators.find(VX_PORT)->second, actuators.find(VY_PORT)->second);
	if (fuelAvailable < delta.length()) {
		delta.normalize();
		delta *= fuelAvailable;
		actuators[VX_PORT] = delta.x;
		actuators[VY_PORT] = delta.y;
	}
	return actuators;
}




PortMapping Brain::step(const PortMapping& output)
{
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	if (timestep == 0){
		assert(output.empty());
		res[SCENARIO_PORT] = Brain::scenarioNumber;
		state = WAITING;
	}else if (timestep > 1){
		if (state == WAITING){
			if (operation_list.empty()){
				_step(output);
			}
			if (!operation_list.empty()){
				Operation* oper = operation_list.front();
				state = RUNNING;
				res = oper->step(output);
			}
		} else if (state == RUNNING) {
			Operation* oper = operation_list.front();
			res = oper->step(output);
			if (oper->state == COMPLETE){
				state = WAITING;
				operation_list.pop();
			}
		}
	}

	prevResult = res;
	prevInput = output;

	timestep++;
	return fuelOveruseFailsafe(output, res);
}


