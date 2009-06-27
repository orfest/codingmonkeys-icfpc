#ifndef EXECUTER_H
#define EXECUTER_H

#include "vm.h"
#include "brain.h"
#include "tracer.h"
#include "config.h"

#include <vector>

class Executer{
public:
    Executer(const Config& config);
    virtual ~Executer();
    void run();
    bool nextStep();
    std::vector<pointF> getShipsPositions() const;
    int getShipsNumber() const;
    const PortMapping& getOutput() const { return output; }

private:
    VM* vm;
    Brain* brain;
    Tracer* tracer;
    PortMapping input;
    PortMapping output;
};

#endif //EXECUTER_H