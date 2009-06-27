#include "brain.h"

#include <exception>

#include "brain1.h"
#include "brain2.h"
#include "brain3.h"
#include "brain4.h"

Brain* Brain::getBrain(int problem){
    if (problem == 0){
        return new B1();
    } else {
        throw new std::exception("Unknown problem type");
    }
    return 0;
}
