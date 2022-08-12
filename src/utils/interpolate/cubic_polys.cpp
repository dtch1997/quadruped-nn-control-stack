// #include "utils/interpolate/cubic_polys.hpp"
// 
// using namespace std;
// using namespace Eigen;
// 
// namespace legged_robot
// {
// 
// CubicPolys::CubicPolys()
// {
//     current_time_ = 0;
//     time_step_ = 1e-3;
// }
// 
// CubicPolys::CubicPolys ( const Eigen::MatrixXd &points, const double &time_step, const double &current_time, const bool &absolute_time )
// {
//     Reset ( points, time_step, current_time, absolute_time );
// }
// 
// CubicPolys::~CubicPolys()
// {
// 
// }
// 
// bool CubicPolys::Reset ( const Eigen::MatrixXd &points, const double &time_step, const double &current_time, const bool &absolute_time )
// {
//     SetCurrentTime ( current_time );
//     SetTimeStep ( time_step );
// 
//     if ( points.cols() != 3 ) {
//         std::cerr << "The column of motion points should be 3!" << std::endl;
//         return false;
//     }
// 
//     if ( points.rows() < 2 ) {
//         std::cerr << "There should be at least two points in the CubicPolys!" << std::endl;
//         return false;
//     }
// 
//     times_.resize ( points.rows() );
//     times_[0] = 0;
//     for ( unsigned int i = 1; i < times_.size(); i++ ) {
//         if ( absolute_time ) {
//             times_ [i] = points ( i, 0 );
//         } else {
//             times_[i] = times_[i-1] + points ( i, 0 );
//         }
//     }
// 
//     coefs_.clear();
//     MatrixXd coef = MatrixXd::Zero ( 3, 4 );
//     coefs_.push_back ( coef );
//     double t;
//     for ( unsigned int i = 1; i < points.rows(); i++ ) {
//         t = times_[i] - times_[i - 1];
//         if ( t <= 0 ) {
//             std::cerr << "The time period between two motion points should be longer than 0!" << std::endl;
//             return false;
//         }
//         coef = GetCoef ( points ( i - 1, 1 ), points ( i - 1, 2 ), points ( i, 1 ), points ( i, 2 ), t );
//         coefs_.push_back ( coef );
//     }
// 
//     return true;
// }
// 
// Eigen::MatrixXd CubicPolys::GetCoef ( const double &y0, const double &dy0, const double &yf, const double &dyf, const double &t )
// {
//     MatrixXd coef ( 3, 4 );
//     Vector4d k;
//     k << y0, dy0, - ( 3 * y0 - 3 * yf + 2 * t * dy0 + t * dyf ) / pow ( t, 2 ), ( 2 * y0 - 2 * yf + t * dy0 + t * dyf ) / pow ( t, 3 );
//     coef.row ( 0 ) = k.transpose();
//     coef.row ( 1 ) << k ( 1 ), 2 * k ( 2 ), 3 * k ( 3 ), 0;
//     coef.row ( 2 ) << 2 * k ( 2 ), 6 * k ( 3 ), 0, 0;
//     return coef;
// }
// 
// Eigen::Vector3d CubicPolys::Output ( const double &t )
// {
//     Vector3d res;
//     double period;
//     for ( unsigned int i = 1; i < times_.size(); i++ ) {
//         if ( t <= times_[i] ) {
//             period = t - times_[i - 1];
//             res = coefs_[i] * Vector4d ( 1, period, pow ( period, 2 ), pow ( period, 3 ) );
//             return res;
//         }
//     }
// //     period = t - times_[times_.size() - 2];
//     period = times_[times_.size() - 1] - times_[times_.size() - 2];
//     res = coefs_[times_.size() - 1] * Vector4d ( 1, period, pow ( period, 2 ), pow ( period, 3 ) );
//     res.tail(2).setZero();
//     return res;
// }
// 
// Eigen::Vector3d CubicPolys::Output()
// {
//     return Output ( current_time_ );
// }
// 
// 
// void CubicPolys::Tick()
// {
//     current_time_ += time_step_;
// }
// 
// Eigen::Vector3d CubicPolys::TickAndOutput()
// {
//     Tick();
//     return Output();
// }
// 
// Eigen::MatrixXd CubicPolys::Output ( const Eigen::VectorXd &t )
// {
//     MatrixXd res ( 3, t.size() );
//     for ( unsigned int i = 0; i < t.size(); i++ ) {
//         res.col ( i ) = Output ( t[i] );
//     }
//     return res;
// }
// 
// double CubicPolys::GetCurrentTime()
// {
//     return current_time_;
// }
// 
// double CubicPolys::GetTimeStep()
// {
//     return time_step_;
// }
// 
// void CubicPolys::SetCurrentTime ( const double &current_time )
// {
//     current_time_ = current_time;
// }
// 
// void CubicPolys::SetTimeStep ( const double &time_step )
// {
//     time_step_ = time_step;
// }
// 
// }
