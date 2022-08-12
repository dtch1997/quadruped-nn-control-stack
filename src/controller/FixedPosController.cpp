#include "controller/FixedPosController.h"

FixedPosController::FixedPosController(Eigen::VectorXf targetPosition, unsigned long max_timesteps):
        _kMaxTimesteps(max_timesteps),
        _targetJntPos(targetPosition),
        _kP(Eigen::VectorXf::Constant(12, 100)),
        _kD(Eigen::VectorXf::Constant(12, 0.5)),
        _timestep(0){
  reset();
  _zeroTarg = Eigen::VectorXf::Zero(12);
};

FixedPosController::~FixedPosController() {};

void FixedPosController::reset() {
  _timestep = 0;
}

bool FixedPosController::isComplete() const { return _timestep >= _kMaxTimesteps; }

CustomTypes::Action FixedPosController::getControlAction(const CustomTypes::State &robotState) {
  _timestep += 1;
  // Ignore the robot state and just return the recorded position / velocity
  return CustomTypes::Action({_targetJntPos,_zeroTarg,_kP,_kD});
};
