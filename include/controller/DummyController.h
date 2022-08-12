#ifndef DUMMY_CONTROLLER_H
#define DUMMY_CONTROLLER_H

// #include "types/CustomTypes.h"
#include "controller/Controller.h"

/* A controller that doesn't do anything. */
class DummyController : public Controller
{
public: 
    DummyController(){}
    ~DummyController(){}
    void reset(){}
    bool isComplete() const { return true;}
    CustomTypes::Action getControlAction(const CustomTypes::State& robotState){ return CustomTypes::zeroAction();}
};

#endif