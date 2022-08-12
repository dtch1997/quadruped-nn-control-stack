#ifndef FIXED_CONTROLLER_H
#define FIXED_CONTROLLER_H

// #include "types/CustomTypes.h"
#include "controller/Controller.h"
#include <cassert>

/* A simple controller that takes care of 
   resetting the robot to a default pose. */
class FixedPosController : public Controller
{
public:
    FixedPosController(Eigen::VectorXf targetPosition, unsigned long max_timesteps);
    ~FixedPosController();
    void reset();
    bool isComplete() const;
    CustomTypes::Action getControlAction(const CustomTypes::State& robotState);

private:
    const unsigned long _kMaxTimesteps;
    unsigned long _timestep;
    const Eigen::VectorXf _targetJntPos;
    const Eigen::VectorXf _kP;
    const Eigen::VectorXf _kD;
    Eigen::VectorXf  _zeroTarg;
    void _initializeTrajectory(const CustomTypes::State& robotState);
};

#endif