#ifndef BRAIN1_H
#define BRAIN1_H

#include "brain.h"

#include "common.h"

class B1 : public Brain{
public:
    B1(){}
    virtual PortMapping initialStep();
    virtual PortMapping step(const PortMapping& output);
    virtual bool finished() const;
};

#endif //BRAIN1_H
