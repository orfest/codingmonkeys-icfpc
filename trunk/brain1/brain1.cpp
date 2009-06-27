#include "brain1.h"

#include <vector>
#include <iostream>
#include <assert.h>

#include "vector.h"

using namespace std;

B1::B1(int sn):Brain(sn) {}

PortMapping B1::step(const PortMapping& output){
	/** /
    cout << output.find(EARTH_X)->second << endl;
    cout << output.find(EARTH_Y)->second << endl;
    cout << output.find(FUEL_PORT)->second << endl;
    cout << output.find(SCORE_PORT)->second << endl;
	cout << output.find(TARGET_RADIUS)->second << endl;
	cout << "------------" << endl;
	//*/
	static int step = 0;
	if (step == 0) {
		r1 = sqrt(pow(output.find(EARTH_X)->second, 2) + pow(output.find(EARTH_Y)->second, 2));
		r2 = output.find(TARGET_RADIUS)->second;
		transferTime = M_PI * sqrt(pow(r1 + r2, 3) / (8 * MU_CONST));
	}

    PortMapping res;
	res[SCENARIO_PORT] = 0;
    res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	if (step == 1) {
		double delta_v1 = sqrt(MU_CONST / r1) * (sqrt(2 * r2 / (r1 + r2)) - 1);
		Vector prevEarth(prevInput.find(EARTH_X)->second, prevInput.find(EARTH_Y)->second);
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

	if (step == ceil(1 + transferTime)) {
		assert(ceil(1 + transferTime) > 1);

		double delta_v2 = sqrt(MU_CONST / r2) * (1 - sqrt(2 * r1 / (r1 + r2)));
		Vector curEarth(output.find(EARTH_X)->second, output.find(EARTH_Y)->second);
		Vector tangent(curEarth.y, -curEarth.x);
		tangent.normalize();
		if (!clockwise)
			tangent = -tangent;

		Vector delta = tangent * delta_v2;
		res[VX_PORT] = delta.x;
		res[VY_PORT] = delta.y;
	}

    prevResult = res;
    prevInput = output;
	step++;
    return res;
}

vector<pointF> B1::getShipsPositions() const{
    pointF p(prevInput.find(EARTH_X)->second, prevInput.find(EARTH_Y)->second);
    vector<pointF> res;
    res.push_back(p);
    return res;
}

int B1::getShipsNumber() const{
    return 1;
}