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
    return true;
}
