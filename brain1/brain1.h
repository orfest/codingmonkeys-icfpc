#ifndef BRAIN1_H
#define BRAIN1_H

#include "brain.h"

#include "common.h"

class B1 : public Brain{
public:
    B1(){}
    virtual std::vector<PortValue> initialStep();
    virtual std::vector<PortValue> step(const std::vector<PortValue>& output);
    virtual bool finished() const;
};

#endif //BRAIN1_H
