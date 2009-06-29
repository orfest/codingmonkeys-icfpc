#include "brain3.h"

#include <vector>
#include <iostream>
#include <assert.h>

using namespace std;

static const double EPS = 10000.0;
static const double TIME_LIMIT = 3e6;
static const double MAX_BURST_RATIO = 500.0;
static const int MAX_STEERING_STEPS = 250;

B3::B3(int sn, VM* vm):Brain(sn, vm){}

PortMapping B3::_step(const PortMapping& output) {
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
	bool skipOtherStateChanges = false;

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

		Vector tangentMy(curMeEarth.y, -curMeEarth.x);
		clockwise = ( Vector::dotProduct(curMyVel, tangentMy) > 0.0 );

		Vector curTargVel = startTargEarth - curTargEarth;
		energy = curTargVel.sqLength() / 2 - MU_CONST / curTargEarth.length();
		double aTo = - MU_CONST / (2 * energy);
		periodTo = ALPHA_CONST * pow(aTo, 1.5);

		Vector tangentTarg(curTargEarth.y, -curTargEarth.x);
		targClockwise = ( Vector::dotProduct(curTargVel, tangentTarg) > 0.0 );

#if 1
		// now we can estimate orbits
		// note that this is used *instead* of actual measuring and automatically bypasses it
		estimateOrbit(curMyVel, -startMeEarth, myAphelion, myPerihelion);
		rFromMin = myAphelion.length();
		rFromMax = myPerihelion.length();
		rFromKnown = true;

		estimateOrbit(curTargVel, -startTargEarth, targAphelion, targPerihelion);
		rToMin = targAphelion.length();
		rToMax = targPerihelion.length();
		rToMaxTargEarth = targPerihelion;
		rToKnown = true;

		// compensate for circle orbits
		if (abs(rFromMax - rFromMin) < EPS)
			rFromMax = rFromMin = curMeEarth.length();
		if (abs(rToMax - rToMin) < EPS)
			rToMax = rToMin = curTargEarth.length();

		state = waitingJumpFrom;
#endif	

#if 0
		// use VM to simulate orbiting and measure orbits
		simulateAndGetOrbits();
		myAphelion = orbits[0].minR;
		myPerihelion = orbits[0].maxR;
		clockwise = orbits[0].clockwise;
		targAphelion = orbits[1].minR;
		targPerihelion = orbits[1].maxR;
		targClockwise = orbits[1].clockwise;

		rFromMin = myAphelion.length();
		rFromMax = myPerihelion.length();
		rFromKnown = true;

		rToMin = targPerihelion.length();
		rToMax = targPerihelion.length();
		rToMaxTargEarth = targPerihelion;
		rToKnown = true;

		state = waitingJumpFrom;
#endif
	}

	if (timestep > 0 && state == measuring) {
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
			if (curTargEarth.length() > rToMax) {
				rToMax = curTargEarth.length();
				rToMaxTargEarth = curTargEarth;
			}
		} else if (timestep == ceil(1 + periodTo)) {
			rToKnown = true;
		}

		if (rFromKnown && rToKnown)
			state = waitingJumpFrom;
	}

#if 0
	// this attempts to switch to pursuit from the beggining if we do not have much time
	if (state == waitingJumpFrom && periodTo + periodFrom >= TIME_LIMIT / 6 &&
		getPhaseDifference(-curMeEarth, -curTargEarth) < 0.1) {

		state = following;
	}
#endif

	if (state == waitingJumpFrom && isPhaseWithinEpsCircleAware(-curMeEarth, myPerihelion, myAphelion)) {
		// jump to circular orbit (rFromMax), and immediately to target circular orbit (rToMax)
		// this effectively transfers to elliptic orbit (rFromMax, rToMax)
		hohmannTransfer(res, rFromMin, rFromMax, true, curMeEarth);
		hohmannTransfer(res, rFromMax, rToMax, false, curMeEarth);

		jumpFromCircPos = -curMeEarth;
		state = jumpedFromCircular;
	}

	if (state == jumpedFromCircular && isPhaseWithinEpsCircleAware(-curMeEarth, -jumpFromCircPos, Vector() )) {
		// transfer to circular (rToMax)
		hohmannTransfer(res, rFromMax, rToMax, true, curMeEarth);

		state = waitingJumpTo;
	}

	if (state == waitingJumpTo && isPhaseWithinEpsCircleAware(-curMeEarth, targPerihelion, targAphelion)) {
		// transfer to elliptic (rToMin, rToMax)
		hohmannTransfer(res, rToMax, rToMin, false, curMeEarth);

		if (!( abs(getPhaseDifference(-curMeEarth, -curTargEarth)) < 0.001) ) {
			if (clockwise == targClockwise) {	// flip direction
				Vector prevMeEarth(prevInput.find(EARTH_X)->second, prevInput.find(EARTH_Y)->second);
				Vector curMyVel = prevMeEarth - curMeEarth;
				Vector curDeltaV(res[VX_PORT], res[VY_PORT]);
				Vector deltaV = - curMyVel * 2 - curDeltaV;

				res[VX_PORT] = deltaV.x;
				res[VY_PORT] = deltaV.y;
				clockwise = !clockwise;
			}

			state = countering;
		} else if (clockwise == targClockwise) {
			state = following;
			skipOtherStateChanges = true;
		} else {
			state = countering;
		}
	}

	if (state == countering && abs(getPhaseDifference(-curMeEarth, -curTargEarth)) < 0.001) {
		// flip direction
		Vector prevMeEarth(prevInput.find(EARTH_X)->second, prevInput.find(EARTH_Y)->second);
		Vector curMyVel = prevMeEarth - curMeEarth;
		Vector curDeltaV(res[VX_PORT], res[VY_PORT]);
		Vector deltaV = - curMyVel * 2 - curDeltaV;

		res[VX_PORT] = deltaV.x;
		res[VY_PORT] = deltaV.y;
		clockwise = !clockwise;
		assert(clockwise == targClockwise);
		state = following;
		skipOtherStateChanges = true;
	}

	if (state == following && !skipOtherStateChanges && 
								(curMeEarth - curTargEarth).length() > 1000) {
		// steer to the target
		Vector prevMeEarth(prevInput.find(EARTH_X)->second, prevInput.find(EARTH_Y)->second);
		Vector prevTargEarth = prevMeEarth - 
							Vector(prevInput.find(SATELLITE_X)->second, prevInput.find(SATELLITE_Y)->second);
		Vector curTargVel = prevTargEarth - curTargEarth;
		Vector curMeVel = prevMeEarth - curMeEarth;
		Vector dirMeTarg = curMeEarth - curTargEarth; 

		Vector deltaV = dirMeTarg / MAX_STEERING_STEPS + (curTargVel - curMeVel);
		if (deltaV.length() > output.find(FUEL_PORT)->second / MAX_BURST_RATIO)
			deltaV = deltaV.normalize() * output.find(FUEL_PORT)->second / MAX_BURST_RATIO;

		res[VX_PORT] = deltaV.x;
		res[VY_PORT] = deltaV.y;
		state = steering;
		steeringSteps = 0;
		skipOtherStateChanges = true;
	}

	if (state == steering && !skipOtherStateChanges) {
		// if we steer away - return to the pursuit
		if (steeringSteps++ > MAX_STEERING_STEPS && (curMeEarth - curTargEarth).length() > 1000)
			state = following;
	}

	if (state == steering && !skipOtherStateChanges && 
								(curMeEarth - curTargEarth).length() < 500) {
		// remove steering velocity component and continue pursuit
		Vector prevMeEarth(prevInput.find(EARTH_X)->second, prevInput.find(EARTH_Y)->second);
		Vector curMyVel = prevMeEarth - curMeEarth;

		Vector prevTargEarth = prevMeEarth - Vector(prevInput.find(SATELLITE_X)->second, prevInput.find(SATELLITE_Y)->second);
		Vector curTargVel = prevTargEarth - curTargEarth;

		Vector deltaV = curTargVel - curMyVel;
		res[VX_PORT] = deltaV.x;
		res[VY_PORT] = deltaV.y;
		state = following;
		skipOtherStateChanges = true;
	}

    prevResult = res;
    prevInput = output;
	timestep++;
    return fuelOveruseFailsafe(output, res);
}


vector<pointF> B3::getShipsPositions(const PortMapping& output) const {
	vector<pointF> res;
    pointF p(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);
    res.push_back(p);
    pointF target( p.first + output.find(TARGET_X)->second, p.second + output.find(TARGET_Y)->second   );
    res.push_back(target);
	
	/** /
	double alpha = (timestep % 2001) / 2000.0;
	res.push_back(myAphelion * (1 - alpha) + myPerihelion * alpha);
	//res.push_back(myPerihelion * (1 - alpha) + myAphelion * alpha);
	res.push_back(targAphelion * (1 - alpha) + targPerihelion * alpha);
	//res.push_back(targPerihelion * (1 - alpha) + targAphelion * alpha);
	//*/

    return res;
}

int B3::getShipsNumber() const {
    return 2; //  +2;
}

void B3::hohmannTransfer(PortMapping & actuators, double fromR, double toR, 
										bool toCircular, Vector curMeEarth) const {
	assert( toCircular || abs(curMeEarth.length() - fromR) < EPS);
	assert(!toCircular || abs(curMeEarth.length() -   toR) < EPS);

	double delta_v;
	if (toCircular)
		delta_v = sqrt(MU_CONST / toR) * (1 - sqrt(2 * fromR / (fromR + toR)));
	else
		delta_v = sqrt(MU_CONST / fromR) * (sqrt(2 * toR / (fromR + toR)) - 1);

	Vector tangent(curMeEarth.y, -curMeEarth.x);
	tangent.normalize();
	if (!clockwise)
		tangent = -tangent;

	Vector delta = tangent * delta_v;
	actuators[VX_PORT] += delta.x;
	actuators[VY_PORT] += delta.y;
}
