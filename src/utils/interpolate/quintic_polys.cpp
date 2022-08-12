#include "utils/interpolate/quintic_polys.hpp"

using namespace std;
using namespace Eigen;

namespace legged_robot
{

QuinticPolys::QuinticPolys()
{
    current_time_ = 0;
    time_step_ = 1e-3;
}

QuinticPolys::QuinticPolys ( const Eigen::MatrixXd &points, const double &time_step, const double &current_time, const bool &absolute_time )
{
    Reset ( points, time_step, current_time, absolute_time );
}

QuinticPolys::~QuinticPolys()
{

}

bool QuinticPolys::Reset ( const Eigen::MatrixXd &points, const double &time_step, const double &current_time, const bool &absolute_time)
{
    SetCurrentTime ( current_time );
    SetTimeStep ( time_step );

    if ( points.cols() != 4 ) {
        std::cerr << "The column of motion points should be 3!" << std::endl;
        return false;
    }

    if ( points.rows() < 2 ) {
        std::cerr << "There should be at least two points in the QuinticPolys!" << std::endl;
        return false;
    }
    
    times_.resize ( points.rows() );
    times_[0] = 0;
    for ( unsigned int i = 1; i < times_.size(); i++ ) {
        if ( absolute_time ) {
            times_ [i] = points ( i, 0 );
        } else {
            times_[i] = times_[i-1] + points ( i, 0 );
        }
    }

    coefs_.clear();
    MatrixXd coef = MatrixXd::Zero ( 3, 6 );
    coefs_.push_back ( coef );
    double t;
    for ( unsigned int i = 1; i < points.rows(); i++ ) {
        t = times_[i] - times_[i - 1];
        if ( t <= 0 ) {
            std::cerr << "The time period between two motion points should be longer than 0!" << std::endl;
            return false;
        }
        coef = GetCoef ( Vector3d ( points ( i - 1, 1 ), points ( i - 1, 2 ), points ( i-1, 3 ) ), Vector3d ( points ( i, 1 ), points ( i, 2 ), points ( i, 3 ) ), t );
        coefs_.push_back ( coef );
    }

    return true;
}

Eigen::MatrixXd QuinticPolys::GetCoef ( const Vector3d &start, const Vector3d &end, const double &duration )
{
    MatrixXd coeffs ( 3, 6 );
    VectorXd k ( 6 );
    k[0] = -0.5 * ( 12 * start[0] - 12 * end[0] + 6 * start[1] * duration + 6 * end[1] * duration + start[2] * pow ( duration, 2 ) - end[2] * pow ( duration, 2 ) ) / pow ( duration, 5 );
    k[1] = 0.5 * ( 30 * start[0] - 30 * end[0] + 16 * start[1] * duration + 14 * end[1] * duration + 3 * start[2] * pow ( duration, 2 ) - 2 * end[2] * pow ( duration, 2 ) ) / pow ( duration, 4 );
    k[2] = -0.5 * ( 20 * start[0] - 20 * end[0] + 12 * start[1] * duration + 8 * end[1] * duration + 3 * start[2] * pow ( duration, 2 ) - end[2] * pow ( duration, 2 ) ) / pow ( duration, 3 );
    k[3] = 0.5 * start[2];
    k[4] = start[1];
    k[5] = start[0];

    coeffs.row ( 0 ) = k.transpose();
    coeffs.row ( 1 ) << 0, 5 * k ( 0 ), 4 * k ( 1 ), 3 * k ( 2 ), 2 * k ( 3 ), k ( 4 );
    coeffs.row ( 2 ) << 0, 0, 20 * k ( 0 ), 12 * k ( 1 ), 6 * k ( 2 ), 2 * k ( 3 );
    return coeffs;
}

Eigen::Vector3d QuinticPolys::Output ( const double &t )
{
    Vector3d res;
    double period;
    VectorXd tt ( 6 );
    for ( unsigned int i = 1; i < times_.size(); i++ ) {
        if ( t <= times_[i] ) {
            period = t - times_[i - 1];
            tt << pow ( period, 5 ), pow ( period, 4 ), pow ( period, 3 ), pow ( period, 2 ), period, 1;
            res = coefs_[i] * tt;
            return res;
        }
    }
//     period = t - times_[times_.size() - 2];
    period = times_[times_.size() - 1] - times_[times_.size() - 2];
    tt << pow ( period, 5 ), pow ( period, 4 ), pow ( period, 3 ), pow ( period, 2 ), period, 1;
    res = coefs_[times_.size() - 1] * tt;
    res.tail(2).setZero();
    return res;
}

Eigen::Vector3d QuinticPolys::Output()
{
    return Output ( current_time_ );
}


void QuinticPolys::Tick()
{
    current_time_ += time_step_;
}

Eigen::Vector3d QuinticPolys::TickAndOutput()
{
    Tick();
    return Output();
}

Eigen::MatrixXd QuinticPolys::Output ( const Eigen::VectorXd &t )
{
    MatrixXd res ( 3, t.size() );
    for ( unsigned int i = 0; i < t.size(); i++ ) {
        res.col ( i ) = Output ( t[i] );
    }
    return res;
}

double QuinticPolys::GetCurrentTime()
{
    return current_time_;
}

double QuinticPolys::GetTimeStep()
{
    return time_step_;
}

void QuinticPolys::SetCurrentTime ( const double &current_time )
{
    current_time_ = current_time;
}

void QuinticPolys::SetTimeStep ( const double &time_step )
{
    time_step_ = time_step;
}

}
