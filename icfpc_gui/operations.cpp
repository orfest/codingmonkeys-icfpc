#include "brain.h"
#include "operations.h"
#include <exception>
#include <iostream>
#include <assert.h>

#include "vector.h"

PortMapping Hohman::step(const PortMapping& output){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	if (timestep == 0) {
		state = RUNNING;
		r1 = sqrt(pow(target.minR.x, 2) + pow(target.minR.y, 2));
		r2 = sqrt(pow(target.maxR.x, 2) + pow(target.maxR.y, 2));

		transferTime = M_PI * sqrt(pow(r1 + r2, 3) / (8 * MU_CONST));
		double delta_v1 = sqrt(MU_CONST / r1) * (sqrt(2 * r2 / (r1 + r2)) - 1);
		Vector prevEarth(Brain::prevInput.find(EARTH_X)->second, Brain::prevInput.find(EARTH_Y)->second);
		Vector curEarth(output.find(EARTH_X)->second, output.find(EARTH_Y)->second);
		Vector moveDir = prevEarth - curEarth;
		Vector tangent(curEarth.y, -curEarth.x);
		tangent.normalize();
		clockwise = ( Vector::dotProduct(moveDir, tangent) > 0.0 );
		if (!clockwise)
			tangent = -tangent;

		Vector delta = tangent * delta_v1;
		res[VX_PORT] = delta.x;
		res[VY_PORT] = delta.y;
	}

	if (timestep == ceil(transferTime)) {
		assert(ceil(transferTime) > 1);

		double delta_v2 = sqrt(MU_CONST / r2) * (1 - sqrt(2 * r1 / (r1 + r2)));
		Vector curEarth(output.find(EARTH_X)->second, output.find(EARTH_Y)->second);
		Vector tangent(curEarth.y, -curEarth.x);
		tangent.normalize();
		if (!clockwise)
			tangent = -tangent;

		Vector delta = tangent * delta_v2;
		res[VX_PORT] = delta.x;
		res[VY_PORT] = delta.y;
		state = COMPLETE;
	}

	timestep++;
	return res;
}

PortMapping CircleToElliptic::step(const PortMapping& output){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	if (timestep == 0) {
		state = RUNNING;
		r1 = sqrt(pow(target.minR.x, 2) + pow(target.minR.y, 2));
		r2 = sqrt(pow(target.maxR.x, 2) + pow(target.maxR.y, 2));

		transferTime = M_PI * sqrt(pow(r1 + r2, 3) / (8 * MU_CONST));
		double delta_v1 = sqrt(MU_CONST / r1) * (sqrt(2 * r2 / (r1 + r2)) - 1);
		Vector prevEarth(Brain::prevInput.find(EARTH_X)->second, Brain::prevInput.find(EARTH_Y)->second);
		Vector curEarth(output.find(EARTH_X)->second, output.find(EARTH_Y)->second);
		Vector moveDir = prevEarth - curEarth;
		Vector tangent(curEarth.y, -curEarth.x);
		tangent.normalize();
		clockwise = ( Vector::dotProduct(moveDir, tangent) > 0.0 );
		if (!clockwise)
			tangent = -tangent;

		Vector delta = tangent * delta_v1;
		res[VX_PORT] = delta.x;
		res[VY_PORT] = delta.y;
	}

	if (timestep == ceil(transferTime)) {
		assert(ceil(transferTime) > 1);
		state = COMPLETE;
	}

	timestep++;
	return res;
}


double CircleToElliptic::GetTransferTime()
{
	r1 = sqrt(pow(target.minR.x, 2) + pow(target.minR.y, 2));
	r2 = sqrt(pow(target.maxR.x, 2) + pow(target.maxR.y, 2));

	return M_PI * sqrt(pow(r1 + r2, 3) / (8 * MU_CONST));
}

PortMapping EllipticToCircle::step(const PortMapping& output){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	if (timestep == 0) {
		state = RUNNING;
		r2 = sqrt(pow(target.minR.x, 2) + pow(target.minR.y, 2));
		r1 = sqrt(pow(target.maxR.x, 2) + pow(target.maxR.y, 2));


		double delta_v2 = sqrt(MU_CONST / r2) * (1 - sqrt(2 * r1 / (r1 + r2)));
		Vector prevEarth(Brain::prevInput.find(EARTH_X)->second, Brain::prevInput.find(EARTH_Y)->second);
		Vector curEarth(output.find(EARTH_X)->second, output.find(EARTH_Y)->second);
		Vector moveDir = prevEarth - curEarth;
		Vector tangent(curEarth.y, -curEarth.x);
		tangent.normalize();
		clockwise = ( Vector::dotProduct(moveDir, tangent) > 0.0 );
		if (!clockwise)
			tangent = -tangent;

		Vector delta = tangent * delta_v2;
		res[VX_PORT] = delta.x;
		res[VY_PORT] = delta.y;
		transferTime = M_PI * sqrt(pow(r1 + r2, 3) / (8 * MU_CONST));
	}
	if (timestep == ceil(transferTime)) {
		assert(ceil(transferTime) > 1);
		state = COMPLETE;
	}

	timestep++;
	return res;
}


PortMapping FreeFly::step(const PortMapping& output){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	if (timestep == ceil(transferTime)) {
		assert(ceil(transferTime) > 1);
		state = COMPLETE;
	}

	timestep++;
	return res;
}
