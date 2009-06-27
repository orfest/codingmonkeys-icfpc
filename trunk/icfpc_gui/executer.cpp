#include "executer.h"

Executer::Executer(const Config& config):vm(0),brain(0),tracer(0),timestep(0){
    vm = new VM(config.program_file);
    tracer = new Tracer(config.trace_output, config.scenario_number);
    brain = Brain::getBrain(config.problem);
    input = brain->initialStep();
}

Executer::~Executer(){
    delete vm;
    delete tracer;
    delete brain;
}

void Executer::run(){
    while (nextStep());
    tracer->dump(timestep);
}

bool Executer::nextStep(){
    output = vm->step(input);
    input = brain->step(output);
    tracer->add(input, timestep);
    timestep++;
    return !(brain->finished());
}

std::vector<pointF> Executer::getShipsPositions() const{
    return brain->getShipsPositions();
}

int Executer::getShipsNumber() const{
    return brain->getShipsNumber();
}
