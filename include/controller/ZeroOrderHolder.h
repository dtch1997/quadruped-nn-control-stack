#ifndef ZERO_ORDER_HOLDER_H
#define ZERO_ORDER_HOLDER_H

// #include "types/CustomTypes.h"
#include "controller/Controller.h"
#include <memory>

class ZeroOrderHolder : public Controller 
{
public: 
    ZeroOrderHolder(
        std::shared_ptr<Controller> controller,
        size_t hold_frames
    );
    void reset() override;
    bool isComplete() const override;
    CustomTypes::Action getControlAction(const CustomTypes::State& robotState) override;
    void sendTime(double t) override;
    void setBaseVelTarget(Vec3<float> lin, Vec3<float> ang) override;

private:
    std::shared_ptr<Controller> _controller;
    const size_t _hold_frames;
    size_t _timestep;
    bool _isComplete;
    bool _isFirstVisit;

    CustomTypes::Action _curr_action;

};

#endif