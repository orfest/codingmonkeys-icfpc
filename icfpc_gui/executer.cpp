#include "executer.h"

Executer::Executer(const Config& conf):config(conf),vm(0),brain(0),tracer(0),timestep(0){
    vm = new VM(config.program_file);
    tracer = new Tracer(config.trace_output, config.scenario_number);
    brain = Brain::getBrain(config.problem, config.scenario_number);
}

Executer::~Executer(){
    delete vm;
    delete tracer;
    delete brain;
}

void Executer::run(){
    while (nextStep())
				    	;
}

bool Executer::nextStep(){
    input = brain->step(output);        //at the very first invokation output should be empty
    tracer->add(input, timestep);
    output = vm->step(input);
    timestep++;
    if (brain->finished(output)){
        tracer->dump(timestep);
        return false;
    }
    return true;
}

std::vector<pointF> Executer::getShipsPositions() const{
    return brain->getShipsPositions();
}

int Executer::getShipsNumber() const{
    return brain->getShipsNumber();
}
