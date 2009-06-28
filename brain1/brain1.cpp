#include "brain1.h"

#include <vector>
#include <queue>
#include <iostream>
#include <assert.h>

#include "vector.h"

using namespace std;

B1::B1(int sn):Brain(sn),step(0) {}

PortMapping B1::_step(const PortMapping& output){
    PortMapping res;
	res[SCENARIO_PORT] = 0;
    res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	if (step == 0){
		static Hohman hoh_transfer;
		Orbit target;
		target.minR.x = output.find(EARTH_X)->second;
		target.minR.y = output.find(EARTH_Y)->second;
		target.maxR.x = -output.find(TARGET_RADIUS)->second;		
		target.maxR.y = 0;
		hoh_transfer.SetTarget(target);
		operation_list.push(static_cast<Operation*>(&hoh_transfer));

	}
	step++;
	return res;
}

vector<pointF> B1::getShipsPositions() const{
    pointF p(-prevInput.find(EARTH_X)->second, -prevInput.find(EARTH_Y)->second);
    vector<pointF> res;
    res.push_back(p);
    return res;
}

int B1::getShipsNumber() const{
    return 1;
}