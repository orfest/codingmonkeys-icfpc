#include "brain.h"

#include <exception>
#include <iostream>
#include <assert.h>
#include <algorithm>

#include "brain1.h"
#include "brain2.h"
#include "brain2_2.h"
#include "brain3.h"
#include "brain3_3.h"
#include "brain4.h"
#include "vector.h"

#include "vm.h"

using namespace std;

PortMapping Brain::prevInput = PortMapping();
PortMapping Brain::prevResult = PortMapping();
const double Brain::fuelEps = 1e-3;

Brain::Brain(int sn, VM* vm_):scenarioNumber(sn),timestep(0),vm(vm_){}

static double eps = 1;

Brain* Brain::getBrain(int problem, int scenarioNumber, VM* vm){
    if (problem == 0) {
        return new B1(scenarioNumber, vm);
	} else if (problem == 1) {
        return new B2_2(scenarioNumber, vm);
    } else if (problem == 2) {
        return new B3_3(scenarioNumber, vm);
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
		delta *= fuelAvailable-fuelEps;
		actuators[VX_PORT] = delta.x;
		actuators[VY_PORT] = delta.y;
	}
	return actuators;
}


PortMapping Brain::step(const PortMapping& output) {
	// hack for Brain3 implementation and Brain4 stub !!!
    if (scenarioNumber / 1000 == 4) {
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
		do {
			if (state == COMPLETE)
				state = WAITING;
			if (state == WAITING){
				if (operation_list.empty()){
					_step(output);
				}
				if (!operation_list.empty()){
					Operation* oper = operation_list.front();
					state = RUNNING;
					PortMapping tmp = oper->step(output);
					res[VX_PORT] += tmp[VX_PORT];
					res[VY_PORT] += tmp[VY_PORT];
				}
			} else if (state == RUNNING) {
				Operation* oper = operation_list.front();
				res = oper->step(output);
				if (oper->state == COMPLETE){
					state = COMPLETE;
					operation_list.pop();
				}
			}
		} while (state == COMPLETE);
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
    vector<double> maxDist(numShips, -1e20);
    vector<double> minDist(numShips, 1e20);
    vector<int> maxDistTime(numShips, -1);
    vector<int> minDistTime(numShips, -1);
    vector<int> done(numShips, 0);
    vector<double> startPolarAngle(numShips);
    vector<Vector> prev(numShips);

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
            if (t == 0){
                startPolarAngle[i] = getPolarAngle(pos);
            }
            double dist = pos.length();
            if (dist > maxDist[i] + eps){
                maxDist[i] = dist;
                maxDistTime[i] = t;
                orbits[i].maxR = pos;
            }
            if (dist < minDist[i] - eps){
                minDist[i] = dist;
                minDistTime[i] = t;
                orbits[i].minR = pos;
            }
            if (t > 1000){
                double curPolarAngle = getPolarAngle(pos);
                double diffPolarAngle = curPolarAngle - startPolarAngle[i];
                while (diffPolarAngle < -M_PI) diffPolarAngle += 2*M_PI;
                while (diffPolarAngle > M_PI) diffPolarAngle -= 2*M_PI;
                if (abs(diffPolarAngle) < 0.002){
                    done[i] = 1;
                    toexamine--;
                    orbits[i].clockwise = isClockwise(pos, prev[i]);
                }
            }
            prev[i] = pos;
        }
        if (t == 0){
            res[SCENARIO_PORT] = 0;
        }
    }
    int i = 1;
}

bool Brain::isClockwise(const Vector& newPosition, const Vector& prevPosition) const{
    Vector move(newPosition-prevPosition);
    
    Vector tangent(newPosition.y, -newPosition.x);      // turned 90 clockwise
	bool clockwise = ( Vector::dotProduct(move, tangent) > 0.0 );
    return clockwise;
}

double Brain::getPolarAngle(const Vector& v) const {
    Vector norm(v);
    norm.normalize();
    double res = acos(norm.x);
    if (norm.y < 0){
        res = -res;
    }
    return res;
}
