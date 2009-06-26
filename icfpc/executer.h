#ifndef EXECUTER_H
#define EXECUTER_H

#include "vm.h"
#include "brain.h"
#include "tracer.h"
#include "config.h"

class Executer{
public:
    Executer(const Config& config):vm(0),brain(0),tracer(0){
        vm = new VM(config.program_file);
        brain = Brain::getBrain(config.problem);
        tracer = new Tracer(config.trace_output);
    }
    bool run();

private:
    bool init();

    VM* vm;
    Brain* brain;
    Tracer* tracer;
};

#endif //EXECUTER_H