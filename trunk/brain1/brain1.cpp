#include "brain1.h"

#include <vector>

using namespace std;

PortMapping B1::initialStep(){
    return PortMapping();
}

PortMapping B1::step(const PortMapping& output){
    return PortMapping();
}

bool B1::finished() const{
    static int cnt = 50;
    cnt--;
    return cnt <= 0;
}

vector<pointF> B1::getShipsPositions() const{
    static int cnt = 15;
    vector<pointF> res;
    res.push_back(pointF(10000000, 500000*cnt));
    cnt--;
    return res;
}

int B1::getShipsNumber() const{
    return 1;
}