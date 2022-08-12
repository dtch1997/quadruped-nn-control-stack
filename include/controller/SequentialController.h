#ifndef SEQUENTIAL_CONTROLLER_H
#define SEQUENTIAL_CONTROLLER_H

// #include "types/CustomTypes.h"
#include "controller/Controller.h"
#include <memory>
#include <vector>

/*  Controller that executes multiple controllers in sequence */ 
class SequentialController : public Controller 
{
public: 
    explicit SequentialController(std::vector<std::shared_ptr<Controller>>  controllers);
    void reset() override;
    bool isComplete() const override;
    CustomTypes::Action getControlAction(const CustomTypes::State& robotState) override;
    void sendTime(double t) override;
    void setBaseVelTarget(Vec3<float> lin, Vec3<float> ang) override;

private:
    const std::vector<std::shared_ptr<Controller>> _controllers;

};

#endif