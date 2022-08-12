#include "utils/interpolate/linear_pose_interpolator.hpp"

using namespace std;
using namespace Eigen;

namespace legged_robot
{

LinearPoseInterpolator::LinearPoseInterpolator()
{
}

LinearPoseInterpolator::LinearPoseInterpolator ( const vector<double> &times, const vector<Affine3d> &points, const double &time_step, const double &current_time, const bool &absolute_time )
{
    Reset ( times, points, time_step, current_time, absolute_time );
}

LinearPoseInterpolator::~LinearPoseInterpolator()
{

}

bool LinearPoseInterpolator::Reset ( const vector<double> &times, const vector<Affine3d> &points, const double &time_step, const double &current_time, const bool &absolute_time )
{
    SetCurrentTime ( current_time );
    SetTimeStep ( time_step );

    if ( points.size() != times.size() ) {
        std::cerr << "The sizes of times and points are not consistent in the MatrixSlerps!" << std::endl;
        return false;
    }

    if ( points.size() < 2 ) {
        std::cerr << "There should be at least two points in the MatrixSlerps!" << std::endl;
        return false;
    }

    poses_ = points;
    times_ = times;
    if ( !absolute_time ) {
        times_[0] = 0;
        for ( unsigned int i = 1; i < times_.size(); i++ ) {
            times_[i] += times_[i-1];
        }
    }

    return true;
}

Affine3d LinearPoseInterpolator::Output ( const double &t )
{
    Affine3d res; 
    for ( unsigned int i = 1; i < times_.size(); i++ ) {
        if ( t <= times_[i] ) {
            double percent = ( t - times_[i-1] ) / ( times_[i] - times_[i-1] );
	    res.translation() = (1-percent)*poses_[i-1].translation() + percent*poses_[i].translation();
	    Quaterniond q0(poses_[i-1].linear());
	    Quaterniond q1(poses_[i].linear());
	    res.linear() = q0.slerp(percent, q1).toRotationMatrix();
            return res;
        }
    }
    res = poses_.back();
    return res;
}

Affine3d LinearPoseInterpolator::Output()
{
    return Output ( GetCurrentTime() );
}

void LinearPoseInterpolator::Tick()
{
    current_time_ += step_time_;
}

Affine3d LinearPoseInterpolator::TickAndOutput()
{
    Tick();
    return Output();
}

vector<Affine3d> LinearPoseInterpolator::Output ( const std::vector<double> &t )
{
    vector<Affine3d> res;
    for ( auto &it:t ) {
        res.push_back ( Output ( it ) );
    }
    return res;
}

double LinearPoseInterpolator::GetCurrentTime()
{
    return current_time_;
}

double LinearPoseInterpolator::GetTimeStep()
{
    return step_time_;
}

void LinearPoseInterpolator::SetCurrentTime ( const double &current_time )
{
    current_time_ = current_time;
}

void LinearPoseInterpolator::SetTimeStep ( const double &time_step )
{
    step_time_ = time_step;
}

bool LinearPoseInterpolator::IsDone ()
{
    return current_time_>times_.back();
}

}
