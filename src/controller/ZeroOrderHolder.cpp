#include "controller/ZeroOrderHolder.h"

ZeroOrderHolder::ZeroOrderHolder
(
    std::shared_ptr<Controller> controller,
    size_t hold_frames
):
    _controller(controller), 
    _hold_frames(hold_frames),
    _timestep(0),
    _curr_action(CustomTypes::zeroAction()),
    _isComplete(_controller->isComplete())
{
    reset();
};

void ZeroOrderHolder::reset()
{
    _controller->reset();
    _isFirstVisit = true;
    _isComplete = false;
    _curr_action = CustomTypes::zeroAction();
    _timestep = 0;
}

bool ZeroOrderHolder::isComplete() const{return _isComplete;}

void ZeroOrderHolder::sendTime(double t) {
  _controller->sendTime(t);
  simulationTime = t;
};
CustomTypes::Action ZeroOrderHolder::getControlAction(const CustomTypes::State& robotState)
{   

    if (_timestep == 0) {
        _curr_action = _controller->getControlAction(robotState);
        _isComplete = _controller->isComplete();
        _timestep =_hold_frames;
    }

    _timestep -= 1;
    return _curr_action;
}


void ZeroOrderHolder::setBaseVelTarget(Vec3<float> lin, Vec3<float> ang)  {
  _controller->setBaseVelTarget(lin, ang);
}