#include "utils/interpolate/matrix_slerps.hpp"

using namespace std;
using namespace Eigen;

namespace legged_robot
{

MatrixSlerps::MatrixSlerps()
{
    current_time_ = 0;
    time_step_ = 1e-3;
}

MatrixSlerps::MatrixSlerps ( const vector<double> times, const vector<Matrix3d> &points, const double &time_step, const double &current_time, const bool &absolute_time )
{
    Reset ( times, points, time_step, current_time, absolute_time );
}

MatrixSlerps::~MatrixSlerps()
{

}

bool MatrixSlerps::Reset ( const vector<double> times, const vector<Matrix3d> &points, const double &time_step, const double &current_time, const bool &absolute_time )
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

    times_.resize ( times.size() );
    times_[0] = 0;
    coefs_.clear();
    coefs_.push_back ( Quaterniond ( points[0] ) );
    for ( unsigned int i = 1; i < times_.size(); i++ ) {
        if ( absolute_time ) {
            times_ [i] = times[i];
        } else {
            times_[i] = times_[i-1] + times[i];
        }
        coefs_.push_back ( Quaterniond ( points[i] ) );
    }
    return true;
}

Eigen::Matrix3d MatrixSlerps::Output ( const double &t )
{
    for ( unsigned int i = 1; i < times_.size(); i++ ) {
        if ( t <= times_[i] ) {
            double percent = ( t - times_[i-1] ) / ( times_[i] - times_[i-1] );
            return coefs_[i-1].slerp ( percent, coefs_[i] ).toRotationMatrix();
        }
    }
    return coefs_.back().toRotationMatrix();
}

Eigen::Matrix3d MatrixSlerps::Output()
{
    return Output ( current_time_ );
}


void MatrixSlerps::Tick()
{
    current_time_ += time_step_;
}

Eigen::Matrix3d MatrixSlerps::TickAndOutput()
{
    Tick();
    return Output();
}

vector<Matrix3d> MatrixSlerps::Output ( const std::vector<double> &t )
{
    vector<Matrix3d> res;
    for ( unsigned int i = 0; i < t.size(); i++ ) {
        res.push_back ( Output ( t[i] ) );
    }
    return res;
}

double MatrixSlerps::GetCurrentTime()
{
    return current_time_;
}

double MatrixSlerps::GetTimeStep()
{
    return time_step_;
}

void MatrixSlerps::SetCurrentTime ( const double &current_time )
{
    current_time_ = current_time;
}

void MatrixSlerps::SetTimeStep ( const double &time_step )
{
    time_step_ = time_step;
}

}


