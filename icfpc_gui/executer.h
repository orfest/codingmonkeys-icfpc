#ifndef EXECUTER_H
#define EXECUTER_H

#include "vm.h"
#include "brain.h"
#include "tracer.h"
#include "config.h"

#include <vector>

class Executer{
public:
    Executer(const Config& conf);
    virtual ~Executer();
    void run();
    bool nextStep();
    std::vector<pointF> getShipsPositions() const;
    int getShipsNumber() const;
    const PortMapping& getOutput() const { return output; }
    const Config& getConfig() const { return config; }

private:
    Config config;
    VM* vm;
    Brain* brain;
    Tracer* tracer;
    PortMapping input;
    PortMapping output;
    int timestep;
};

#endif //EXECUTER_H