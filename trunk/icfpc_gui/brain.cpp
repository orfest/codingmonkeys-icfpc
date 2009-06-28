#include "brain.h"

#include <exception>
#include <iostream>
#include <assert.h>
#include <algorithm>

#include "brain1.h"
#include "brain2.h"
#include "brain2_2.h"
#include "brain3.h"
#include "brain4.h"
#include "vector.h"

#include "vm.h"

using namespace std;

PortMapping Brain::prevInput = PortMapping();
PortMapping Brain::prevResult = PortMapping();

Brain::Brain(int sn, VM* vm_):scenarioNumber(sn),timestep(0),vm(vm_){}

Brain* Brain::getBrain(int problem, int scenarioNumber, VM* vm){
    if (problem == 0) {
        return new B1(scenarioNumber, vm);
	} else if (problem == 1) {
        return new B2_2(scenarioNumber, vm);
    } else if (problem == 2) {
        return new B3(scenarioNumber, vm);
    } else if (problem == 3) {
        return new B4(scenarioNumber, vm);
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


PortMapping Brain::step(const PortMapping& output) {
    simulateAndGetOrbits();
	// hack for Brain3 implementation and Brain4 stub !!!
    if (scenarioNumber / 1000 == 3 || scenarioNumber / 1000 == 4) {
		return _step(output);
    }

	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	if (timestep == 0){
		assert(output.empty());
		res[SCENARIO_PORT] = Brain::scenarioNumber;
		state = WAITING;

        simulateAndGetOrbits();

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

void Brain::simulateAndGetOrbits(){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

    int numShips = this->getShipsNumber();
    orbits.resize(numShips);
    vector<int> wasMax(numShips, 0);
    vector<int> wasMin(numShips, 0);
    vector<double> curDistance(numShips, -1);
    vector<double> prevDistance(numShips, -1);
    vector<Vector> curPos(numShips);
    vector<Vector> prevPos(numShips);
    vector<int> done(numShips, 0);

    int toexamine = numShips;
    for (int t = 0; toexamine > 0 && t < 3000000; t++){
        if (t == 0){
            res[SCENARIO_PORT] = scenarioNumber;
        }
        PortMapping output = vm->step(res);
        vector<pointF> shipsPositions = this->getShipsPositions(output);
        assert(shipsPositions.size() == orbits.size());
        for (int i = 0; i < numShips; i++){
            if (done[i]) continue;
            Vector pos(shipsPositions[i].first, shipsPositions[i].second);
            double dist = pos.length();
            if (dist > 2){
                if (dist < curDistance[i] && curDistance[i] > prevDistance[i]){
                    assert(!wasMax[i]);
                    wasMax[i] = 1;
                    orbits[i].maxR = curPos[i];
                    if (wasMin[i]){
                        assert(!done[i]);
                        done[i] = 1;
                        toexamine--;
                    }
                }
                if (dist > curDistance[i] && curDistance[i] < prevDistance[i]){
                    assert(!wasMin[i]);
                    wasMin[i] = 1;
                    orbits[i].minR = curPos[i];
                    if (wasMax[i]){
                        assert(!done[i]);
                        done[i] = 1;
                        toexamine--;
                    }
                }
            }
            prevDistance[i] = curDistance[i];
            curDistance[i] = dist;
            prevPos[i] = curPos[i];
            curPos[i] = pos;
        }
        if (t == 0){
            res[SCENARIO_PORT] = 0;
        }
    }
    int i = 1;
}
