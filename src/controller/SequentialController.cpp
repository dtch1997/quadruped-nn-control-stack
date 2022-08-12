#include "controller/SequentialController.h"

#include <utility>

SequentialController::SequentialController(std::vector<std::shared_ptr<Controller>>  controllers):
    _controllers(std::move(controllers))
{}

void SequentialController::reset(){
  for (const auto& ctrl : _controllers){ctrl->reset();}
  _curr_idx = 0;
}
void SequentialController::sendTime(double t) {
  for (const auto& ctrl : _controllers){ctrl->sendTime(t);}
  simulationTime = t;
}
bool SequentialController::isComplete() const {
    return _curr_idx == _controllers.size();
}

CustomTypes::Action SequentialController::getControlAction(const CustomTypes::State& robotState){
    auto action = _controllers[_curr_idx]->getControlAction(robotState);
    if (_controllers[_curr_idx]->isComplete()) _curr_idx += 1;
    return action;
}

void SequentialController::setBaseVelTarget(Vec3<float> lin, Vec3<float> ang)  {
    _controllers[_controllers.size()-1]->setBaseVelTarget(lin, ang);
}