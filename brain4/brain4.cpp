#include "brain4.h"

#include <vector>
#include <iostream>
#include <assert.h>

#include "vector.h"

using namespace std;

B4::B4(int sn):Brain(sn) {}

PortMapping B4::step(const PortMapping& output){
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

    if (timestep == 0){
        assert(output.empty());
        res[SCENARIO_PORT] = Brain::scenarioNumber;
    }

    prevResult = res;
    prevInput = output;
	timestep++;
    return fuelOveruseFailsafe(output, res);
}

vector<pointF> B4::getShipsPositions() const{
    pointF p(-prevInput.find(EARTH_X)->second, -prevInput.find(EARTH_Y)->second);
    vector<pointF> res;
    res.push_back(p);
	pointF station( p.first + prevInput.find(STATION_X)->second, p.second + prevInput.find(STATION_Y)->second   );
    res.push_back(station);

	pointF target;

#define PUSH_TARGETN(n)	\
	target = pointF( p.first + prevInput.find(TARGET##n##_X)->second, p.second + prevInput.find(TARGET##n##_Y)->second   );	\
    res.push_back(target);

	PUSH_TARGETN(0)
	PUSH_TARGETN(1)
	PUSH_TARGETN(2)
	PUSH_TARGETN(3)
	PUSH_TARGETN(4)
	PUSH_TARGETN(5)
	PUSH_TARGETN(6)
	PUSH_TARGETN(7)
	PUSH_TARGETN(8)
	PUSH_TARGETN(9)
	PUSH_TARGETN(10)
	//PUSH_TARGETN(11)	// in spec 11 targets

#undef PUSH_TARGETN(n)

	pointF moon( p.first + prevInput.find(MOON_X)->second, p.second + prevInput.find(MOON_Y)->second   );
    res.push_back(moon);

    return res;
}

int B4::getShipsNumber() const{
    return 2+11+1;	// in spec 2+12+1
}
