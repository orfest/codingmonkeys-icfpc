#include "brain1.h"

#include <vector>
#include <iostream>

using namespace std;

B1::B1(int sn):Brain(sn){}

PortMapping B1::step(const PortMapping& output){
    //cout << output.find(EARTH_X)->second << endl;
    //cout << output.find(EARTH_Y)->second << endl;
    //cout << output.find(FUEL_PORT)->second << endl;
    //cout << output.find(SCORE_PORT)->second << endl;
    PortMapping res;
    res[SCENARIO_PORT] = 0;//Brain::scenarioNumber;
//    static int g = 10000;
    res[VX_PORT] = 0;
 //   g -= 1000;
    res[VY_PORT] = 0;
    prevResult = res;
    prevInput = output;
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