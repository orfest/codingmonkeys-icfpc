#include "brain4.h"

#include <vector>
#include <iostream>
#include <assert.h>

#include "vector.h"

using namespace std;

B4::B4(int sn, VM* vm):Brain(sn, vm) {}

PortMapping B4::_step(const PortMapping& output){
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

vector<pointF> B4::getShipsPositions(const PortMapping& output) const{
    pointF p(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);
    vector<pointF> res;
    res.push_back(p);
	pointF station( p.first + output.find(STATION_X)->second, p.second + output.find(STATION_Y)->second   );
    res.push_back(station);

	for (int i = 0; i < 11; i++) {	
		pointF target( p.first + output.find(TARGETN_X(i))->second, p.second + output.find(TARGETN_Y(i))->second );
		res.push_back(target);
	}

	pointF moon( p.first + output.find(MOON_X)->second, p.second + output.find(MOON_Y)->second   );
    res.push_back(moon);

    return res;
}

int B4::getShipsNumber() const{
    return 2+11+1;
}
