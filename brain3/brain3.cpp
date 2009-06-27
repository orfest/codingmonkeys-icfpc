#include "brain3.h"

#include <vector>
#include <iostream>
#include <assert.h>

using namespace std;

static const double EPS = 10;

B3::B3(int sn):Brain(sn) {}

PortMapping B3::step(const PortMapping& output){
	/** /
    cout << output.find(EARTH_X)->second << endl;
    cout << output.find(EARTH_Y)->second << endl;
    cout << output.find(FUEL_PORT)->second << endl;
    cout << output.find(SCORE_PORT)->second << endl;
	cout << output.find(TARGET_RADIUS)->second << endl;
	cout << "------------" << endl;
	//*/

    PortMapping res;
	res[SCENARIO_PORT] = 0;
    res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	Vector curMeEarth, curTargEarth;

    if (timestep == 0) {
        assert(output.empty());
        res[SCENARIO_PORT] = Brain::scenarioNumber;
		state = measuring;
	} else {
		curMeEarth = Vector(output.find(EARTH_X)->second, output.find(EARTH_Y)->second);
		curTargEarth = curMeEarth - 
			Vector(output.find(SATELLITE_X)->second, output.find(SATELLITE_Y)->second);
	}

	if (timestep == 1) {	// measuring starting positions
		startMeEarth = curMeEarth;
		startTargEarth = curTargEarth;
		rFromMin = rFromMax = curMeEarth.length();
		rFromKnown = false;
		rToMin = rToMax = curTargEarth.length();
		rToKnown = false;
	}

	if (timestep == 2) {	// measuring velocities
		double energy;
		Vector curMyVel = startMeEarth - curMeEarth;
		energy = curMyVel.sqLength() / 2 - MU_CONST / curMeEarth.length();
		double aFrom = - MU_CONST / (2 * energy);
		periodFrom = ALPHA_CONST * pow(aFrom, 1.5);

		Vector curTargVel = startTargEarth - curTargEarth;
		energy = curTargVel.sqLength() / 2 - MU_CONST / curTargEarth.length();
		double aTo = - MU_CONST / (2 * energy);
		periodTo = ALPHA_CONST * pow(aTo, 1.5);
	}

	if (timestep > 0) {
		if (!rFromKnown && timestep < ceil(1 + periodFrom)) {
			if (curMeEarth.length() < rFromMin)
				rFromMin = curMeEarth.length();
			if (curMeEarth.length() > rFromMax)
				rFromMax = curMeEarth.length();
		} else if (timestep == ceil(1 + periodFrom)) {
			rFromKnown = true;
		}

		if (!rToKnown && timestep < ceil(1 + periodTo)) {
			if (curTargEarth.length() < rToMin)
				rToMin = curTargEarth.length();
			if (curTargEarth.length() > rToMax)
				rToMax = curTargEarth.length();
		} else if (timestep == ceil(1 + periodTo)) {
			rToKnown = true;
		}

		if (state == measuring && rFromKnown && rToKnown)
			state = waitingJumpFrom;
	}

	if (state == waitingJumpFrom && abs(rFromMax - curMeEarth.length()) < EPS) {
		// jump to circular orbit, and immediately to target circular orbit
		double delta_v2 = sqrt(MU_CONST / rFromMax) * (1 - sqrt(2 * rFromMin / (rFromMin + rFromMax)));
		Vector tangent(curMeEarth.y, -curMeEarth.x);
		tangent.normalize();
		Vector curVel = curMeEarth - Vector(prevInput.find(EARTH_X)->second, prevInput.find(EARTH_Y)->second);
		clockwise = ( Vector::dotProduct(- curVel, tangent) > 0.0 );
		if (!clockwise)
			tangent = -tangent;
		if (rFromMax > rToMax) { // flip direction here
			delta_v2 = delta_v2 + 2 * curVel.length();
			tangent = -tangent;
			clockwise = !clockwise;
		}

		double delta_v1 = sqrt(MU_CONST / rFromMax) * (sqrt(2 * rToMax / (rFromMax + rToMax)) - 1);

		Vector delta = tangent * (delta_v1 + delta_v2);
		res[VX_PORT] = delta.x;
		res[VY_PORT] = delta.y;
		state = jumpedFromCircular;
	}

	if (state == jumpedFromCircular && abs(rToMax - curMeEarth.length()) < EPS) {
		double delta_v2 = sqrt(MU_CONST / rToMax) * (1 - sqrt(2 * rFromMax / (rFromMax + rToMax)));
		Vector tangent(curMeEarth.y, -curMeEarth.x);
		tangent.normalize();
		Vector curVel = curMeEarth - Vector(prevInput.find(EARTH_X)->second, prevInput.find(EARTH_Y)->second);
		if (!clockwise)
			tangent = -tangent;
		if (rToMax > rFromMax) { // flip direction here
			delta_v2 = delta_v2 + 2 * curVel.length();
			tangent = -tangent;
			clockwise = !clockwise;
		}

		Vector delta = tangent * delta_v2;
		res[VX_PORT] = delta.x;
		res[VY_PORT] = delta.y;
		state = waitingJumpTo;
	}

	// TODO 

    prevResult = res;
    prevInput = output;
	timestep++;
    return fuelOveruseFailsafe(output, res);
}

vector<pointF> B3::getShipsPositions() const{
    pointF p(-prevInput.find(EARTH_X)->second, -prevInput.find(EARTH_Y)->second);
    vector<pointF> res;
    res.push_back(p);
    pointF target( p.first + prevInput.find(TARGET_X)->second, p.second + prevInput.find(TARGET_Y)->second   );
    res.push_back(target);
    return res;
}

int B3::getShipsNumber() const{
    return 2;
}