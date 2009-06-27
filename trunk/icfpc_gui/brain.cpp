#include "brain.h"

#include <exception>
#include <iostream>

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
}

bool Brain::finished(const PortMapping& output) const{
    data_t score = output.find(SCORE_PORT)->second;
    if (score == 0) return false;
    std::cout << 
        "Scenario "     << scenarioNumber << " completed!\n" << 
        "Final score: " << score << "\n" << 
        "Time: "        << timestep << "\n" << 
        "Fuel remaining: " << output.find(FUEL_PORT)->second << std::endl;
    return true;
}
