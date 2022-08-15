#include "utils/interpolate/motion_frame_interpolator.hpp"

namespace legged_robot
{
using namespace Eigen;
using namespace std;

StampedMotionFrameMapInterpolator::StampedMotionFrameMapInterpolator()
{
    current_time_ = 0;
    step_time_ = 0;
    duration_ = 0;
}

StampedMotionFrameMapInterpolator::StampedMotionFrameMapInterpolator ( const StampedMotionFrameMap &start_abstract, const StampedMotionFrameMap &end_abstract, const double &duration, const double &step_time )
{
    current_time_ = 0;
    step_time_ = step_time;
    duration_ = duration;
    
    vector<double> times = {0, duration_};
    vector<MotionFrame> points;
    poly_map_.clear();
    for ( auto &it:start_abstract.motion_frame_map ) {
        points.clear();
	points.push_back(it.second);
	points.push_back(end_abstract.motion_frame_map.at(it.first));
        poly_map_.insert ( pair<string, QuinticPolysPose> ( it.first, QuinticPolysPose ( times, points, step_time_ ) ) );
    }
    start_ = start_abstract;
    end_ = end_abstract;
}

StampedMotionFrameMapInterpolator::~StampedMotionFrameMapInterpolator()
{
}

void StampedMotionFrameMapInterpolator::Tick()
{
    current_time_ += step_time_;
}

StampedMotionFrameMap StampedMotionFrameMapInterpolator::Output()
{
    return Output ( current_time_ );
}

StampedMotionFrameMap StampedMotionFrameMapInterpolator::Output ( const double &time )
{
    StampedMotionFrameMap res;
    
    if ( IsExpired(time) ) {
      res = end_;
    } else {
      res = start_;
    }
    
    for ( auto &it:poly_map_ ) {
      res.motion_frame_map[it.first] = it.second.Output(time);
    }

    return res;
}

StampedMotionFrameMap StampedMotionFrameMapInterpolator::TickAndOutput()
{
    Tick();
    return Output();
}

bool StampedMotionFrameMapInterpolator::IsExpired()
{
    return IsExpired ( current_time_ );
}

bool StampedMotionFrameMapInterpolator::IsExpired ( const double &time )
{
    return ( time >= duration_ - 0.00001 );
}

void StampedMotionFrameMapInterpolator::SetExpired()
{
    current_time_ = duration_;
}

}
