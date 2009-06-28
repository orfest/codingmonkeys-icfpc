#include "brain3_3.h"

#include <vector>
#include <iostream>
#include <assert.h>

#include "operations.h"

using namespace std;

static const double EPS = 0.1;


B3_3::B3_3(int sn, VM* vm):Brain(sn, vm), step(0){}

PortMapping B3_3::_step(const PortMapping& output){
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
	
	if (step == 0){
		Orbit target;
		target.minR = orbits[1].minR;
		target.maxR = orbits[0].minR;
		if (abs(pow(orbits[0].minR.x,2) + pow(orbits[0].minR.y,2)- 
			pow(orbits[0].maxR.x,2) + pow(orbits[0].maxR.y,2)) > 0.000001){
			target.minR = 1000;
		}
		static FreeFlyToOpositPoint op1;
		op1.SetTarget(target);
		operation_list.push(static_cast<Operation*>(&op1));
		static Hohman el1;
		target.minR = orbits[0].minR;
		target.maxR = orbits[1].minR;
		el1.SetTarget(target);
		operation_list.push(static_cast<Operation*>(&el1));
    } else {
        int alp = estimateTimeToPerihelion(target.maxR);
        int i = 1;
	}

    prevResult = res;
    prevInput = output;
	step++;
    return fuelOveruseFailsafe(output, res);
}

vector<pointF> B3_3::getShipsPositions(const PortMapping& output) const {
	pointF p(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);
    vector<pointF> res;
    res.push_back(p);
	pointF target( p.first + output.find(TARGET_X)->second, p.second + output.find(TARGET_Y)->second   );
    res.push_back(target);
    return res;
}

int B3_3::getShipsNumber() const{
    return 2;
}

