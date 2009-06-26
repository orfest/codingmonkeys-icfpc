#include "executer.h"

#include <vector>

using namespace std;

bool Executer::run(){
    vector<PortValue> input = brain->initialStep();
    vector<PortValue> output;
    while (!brain->finished()){
        output = vm->step(input);
        input = brain->step(output);
        tracer->add(input);
    }
    tracer->dump();
    return true;
}
