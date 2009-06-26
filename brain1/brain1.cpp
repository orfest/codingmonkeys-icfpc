#include "brain1.h"

#include <vector>

using namespace std;

vector<PortValue> B1::initialStep(){
    return vector<PortValue>();
}

vector<PortValue> B1::step(const vector<PortValue>& output){
    return vector<PortValue>();
}

bool B1::finished() const{
    return true;
}
