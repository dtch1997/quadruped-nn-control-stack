/*
 * All rights reserved.
 * Copyright (C) 2018 Yangwei You, yangwei.you@foxmail.com
 *
 */


/**
 * @file motion_frame_interpolator.hpp
 * @brief File containing the class to do interpolation between two motion frame maps.
 *
 * @author Yangwei You
 * @date 21 May 2018
 *
 */

#pragma once

#include "types/motion_frame.hpp"
#include "quintic_polys_pose.hpp"


/**
 * @brief namespace for the modules of legged robot
 *
 */
namespace legged_robot
{

/**
 * @brief the class to do interpolation between two motion frame maps
 *
 */
class StampedMotionFrameMapInterpolator
{
public:
    StampedMotionFrameMapInterpolator();
    StampedMotionFrameMapInterpolator(const StampedMotionFrameMap &start_abstract, const StampedMotionFrameMap &end_abstract, const double &duration, const double &step_time);
    ~StampedMotionFrameMapInterpolator();
    
    /**
     * @brief Tick one step of the internal clock
     * 
     */
    void Tick();
    
        
    /**
     * @brief Calculate the value at current time of internal clock
     * 
     * @return StampedMotionFrameMap
     */
    StampedMotionFrameMap Output();
        
    /**
     * @brief Calculate the value at given time
     * 
     * @param time the queried time
     * @return StampedMotionFrameMap
     */
    StampedMotionFrameMap Output(const double &time);
    
    /**
     * @brief Tick one step and then output
     * 
     * @return StampedMotionFrameMap
     */
    StampedMotionFrameMap TickAndOutput();
    
    /**
     * @brief Check if the current time excesses the ending time
     * 
     * @return bool
     */
    bool IsExpired();
    
    /**
     * @brief Check if the given time excesses the ending time
     * 
     * @param time the queried time
     * @return bool
     */
    bool IsExpired(const double &time);
    
    /**
     * @brief Set the status to be expired
     * 
     * @return void
     */
    void SetExpired();

private:
    StampedMotionFrameMap start_;
    StampedMotionFrameMap end_;
    double duration_;
    double current_time_;
    double step_time_;
    std::map<std::string, QuinticPolysPose> poly_map_;
};

}