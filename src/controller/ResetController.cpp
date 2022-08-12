#include "controller/ResetController.h"

ResetController::ResetController
        (
                Eigen::VectorXf finalMotorPosition,
                Eigen::VectorXf midMotorPosition,
                unsigned long max_timesteps,
                float dt
        ) :
        _kMaxTimesteps(max_timesteps),
        _kFinalMotorPosition(finalMotorPosition),
        _kMidMotorPosition(midMotorPosition),
        _kP(Eigen::VectorXf::Constant(12, 100)),
        _kD(Eigen::VectorXf::Constant(12, 0.5)),
        _dt(dt),
        _timestep(0),
        _isFirstVisit(true) {
  assert(_kFinalMotorPosition.rows() == kNumMotors);
  assert(_kMidMotorPosition.rows() == kNumMotors);
  reset();
};

ResetController::~ResetController() {};

void ResetController::reset() {
  _timestep = 0;
  _isFirstVisit = true;
}

bool ResetController::isComplete() const { return _timestep >= _kMaxTimesteps; }

void ResetController::_initializeTrajectory(const CustomTypes::State &robotState) {
  Eigen::VectorXf _initialMotorPosition = robotState.motorPosition;

  float ini[3 * kNumMotors] = {0.};
  float fin[3 * kNumMotors] = {0.};
  float **mid = new float *[1];
  mid[0] = new float[kNumMotors];

//  DBG_INFO("Initializing Standup Traj: ");
//  DBG_INFO("ini:" << _initialMotorPosition.transpose());
//  DBG_INFO("mid:" << _kMidMotorPosition.transpose());
//  DBG_INFO("fin:" << _kFinalMotorPosition.transpose());

  for (size_t i(0); i < kNumMotors; ++i) {
    ini[i] = _initialMotorPosition[i];
    fin[i] = _kFinalMotorPosition[i];
    mid[0][i] = _kMidMotorPosition[i];
  }

  _trajectory.SetParam(ini, fin, mid, _kMaxTimesteps * _dt);

  delete[] mid[0];
  delete[] mid;
}

CustomTypes::Action ResetController::getControlAction(const CustomTypes::State &robotState) {
  if (_isFirstVisit) {
    _initializeTrajectory(robotState);
    _isFirstVisit = false;
  }

  Eigen::VectorXf _targetMotorPosition = Eigen::VectorXf::Zero(12);
  Eigen::VectorXf _targetMotorVelocity = Eigen::VectorXf::Zero(12);
  _trajectory.getCurvePoint(_timestep * _dt, _targetMotorPosition.data());

  _timestep += 1;
  // Ignore the robot state and just return the recorded position / velocity
  return CustomTypes::Action({
                                     _targetMotorPosition,
                                     _targetMotorVelocity,
                                     _kP,
                                     _kD
                             });
};
