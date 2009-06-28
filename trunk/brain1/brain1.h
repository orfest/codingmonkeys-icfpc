#ifndef BRAIN1_H
#define BRAIN1_H

#include "brain.h"

#include "common.h"

class B1 : public Brain{
public:
    B1(int scenarioNumber);
    virtual PortMapping _step(const PortMapping& output);

    virtual std::vector<pointF> getShipsPositions() const;
    virtual int getShipsNumber() const;

private:
    
	int step;
};

#endif //BRAIN1_H
