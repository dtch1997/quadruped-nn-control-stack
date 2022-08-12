#include "utils/interpolate/quintic_polys_pose.hpp"

using namespace std;
using namespace Eigen;

namespace legged_robot
{

QuinticPolysPose::QuinticPolysPose()
{
}

QuinticPolysPose::QuinticPolysPose ( const std::vector<double> &times, const vector<MotionFrame> &points, const double &time_step, const double &current_time, const bool &absolute_time )
{
    Reset ( times, points, time_step, current_time, absolute_time );
}

QuinticPolysPose::~QuinticPolysPose()
{

}

bool QuinticPolysPose::Reset ( const std::vector<double> &times, const vector<MotionFrame> &points, const double &time_step, const double &current_time, const bool &absolute_time )
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

    polys_.clear();
    MatrixXd linear ( times.size(), 4 );
    for ( unsigned int j=0; j<3; j++ ) {
        for ( unsigned int i=0; i < times.size(); i++ ) {
            linear.row ( i ) << times[i], points[i].pose.position[j], points[i].twist.linear[j], points[i].accel.linear[j];
        }
        polys_.push_back ( QuinticPolys ( linear, time_step, current_time, absolute_time ) );
    }

    vector<Matrix3d> rotations;
    for ( unsigned int i=0; i<times.size(); i++ ) {
        rotations.push_back ( points[i].pose.orientation );
    }
    rotation_.Reset ( times, rotations, time_step, current_time, absolute_time );
    
    spinning_vel_ = points[0].spinning_vel;

    return true;
}

MotionFrame QuinticPolysPose::Output ( const double &t )
{
    MotionFrame res;
    for ( unsigned int i=0; i<3; i++ ) {
        res.pose.position[i] = polys_[i].Output ( t ) [0];
        res.twist.linear[i] = polys_[i].Output ( t ) [1];
        res.accel.linear[i] = polys_[i].Output ( t ) [2];
    }
    res.pose.orientation = rotation_.Output ( t );
    res.spinning_vel = spinning_vel_;
    return res;
}

MotionFrame QuinticPolysPose::Output()
{
    return Output ( rotation_.GetCurrentTime() );
}

void QuinticPolysPose::Tick()
{
    for ( auto &it:polys_ ) {
        it.Tick();
    }
    rotation_.Tick();
}

MotionFrame QuinticPolysPose::TickAndOutput()
{
    Tick();
    return Output();
}

vector<MotionFrame> QuinticPolysPose::Output ( const std::vector<double> &t )
{
    vector<MotionFrame> res;
    for ( auto &it:t ) {
        res.push_back ( Output ( it ) );
    }
    return res;
}

double QuinticPolysPose::GetCurrentTime()
{
    return rotation_.GetCurrentTime();
}

double QuinticPolysPose::GetTimeStep()
{
    return rotation_.GetTimeStep();
}

void QuinticPolysPose::SetCurrentTime ( const double &current_time )
{
    for ( auto &it:polys_ ) {
        it.SetCurrentTime ( current_time );
    }
    rotation_.SetCurrentTime ( current_time );
}

void QuinticPolysPose::SetTimeStep ( const double &time_step )
{
    for ( auto &it:polys_ ) {
        it.SetTimeStep ( time_step );
    }
    rotation_.SetTimeStep ( time_step );
}

}
