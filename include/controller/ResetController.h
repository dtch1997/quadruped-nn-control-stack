#ifndef RESET_CONTROLLER_H
#define RESET_CONTROLLER_H

#include "controller/Controller.h"
#include "utils/interpolate/bspline_interp.h"
#include <cassert>

/* A simple controller that takes care of 
   resetting the robot to a default pose. */
class ResetController : public Controller
{
public: 
    ResetController(
        Eigen::VectorXf finalMotorPosition, 
        Eigen::VectorXf midMotorPosition,
        unsigned long max_timesteps, 
        float dt
    );
    ~ResetController();
    void reset();
    bool isComplete() const;
    CustomTypes::Action getControlAction(const CustomTypes::State& robotState);

private:
    const unsigned long _kMaxTimesteps;
    unsigned long _timestep;
    const Eigen::VectorXf _kFinalMotorPosition;
    const Eigen::VectorXf _kMidMotorPosition;
    const Eigen::VectorXf _kP;
    const Eigen::VectorXf _kD;
    const float _dt;

    BS_Basic<float, kNumMotors, 3, 1, 2, 2> _trajectory;
    bool _isFirstVisit;
    void _initializeTrajectory(const CustomTypes::State& robotState);
};

#endif