#include "brain3.h"

#include <vector>
#include <iostream>
#include <assert.h>

#include "vector.h"

using namespace std;

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

    if (timestep == 0){
        assert(output.empty());
        res[SCENARIO_PORT] = Brain::scenarioNumber;
    }
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