#include "brain.h"

#include <exception>

#include "brain1.h"
#include "brain2.h"
#include "brain3.h"
#include "brain4.h"

Brain* Brain::getBrain(int problem, int scenarioNumber){
    if (problem == 0){
        return new B1(scenarioNumber);
    } else {
        throw new std::exception("Unknown problem type");
    }
    return 0;
}

bool Brain::finished(const PortMapping& output) const{
    return (output.find(SCORE_PORT)->second != 0);
}
