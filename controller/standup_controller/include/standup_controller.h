#include "bspline_interp.h"
#include "controller_interface.h"
#include <eigen3/Eigen/Core>

class StandupController : public interfaces::ControllerInterface
{
public:
    int start() override; 
    int step_control() override;
    int stop() override;

private:
    const unsigned long _kMaxTimesteps;
    unsigned long _timestep;
    const Eigen::VectorXf _kFinalMotorPosition;
    const Eigen::VectorXf _kMidMotorPosition;
    const Eigen::VectorXf _kP;
    const Eigen::VectorXf _kD;
    const float _dt;

    BS_Basic<float, 12, 3, 1, 2, 2> _trajectory;
    bool _isFirstVisit;
    void _initializeTrajectory();
};