/*
 * All rights reserved.
 * Copyright (C) 2017 Yangwei You, yangwei.you@foxmail.com
 *
 */


/**
 * @file quintic_polys_pose.hpp
 * @brief File containing the class to construct a vector of quintic polynomials for 6d pose.
 *
 * @author Yangwei You
 * @date 25 Jan 2017
 *
 */

#ifndef QUINTIC_POLYS_POSE_HPP
#define QUINTIC_POLYS_POSE_HPP

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "types/motion_frame.hpp"
#include "quintic_polys.hpp"
#include "matrix_slerps.hpp"

/**
 * @brief namespace for the modules of legged robot
 *
 */
namespace legged_robot
{

/**
 * @brief the class to construct a vector of quintic polynomials for 6d pose
 *
 */
class QuinticPolysPose
{
public:
    QuinticPolysPose();
    
    /**
     * @brief Class Construct
     * 
     * @param points a vector of serial MotionFrame.
     * @param time_step time step for each tick of the internal clock inside the QuinticPolys
     * @param current_time current time of the internal clock inside the QuinticPolys
     */
    QuinticPolysPose(const std::vector<double> &times, const std::vector<MotionFrame> &points, const double &time_step = 1e-3, const double &current_time = 0, const bool &absolute_time = true);
    
    ~QuinticPolysPose();

    /**
     * @brief Reset the quintic polynomials pose
     * 
     * @param points a vector of serial MotionFrame.
     * @param time_step time step for each tick of the internal clock inside the QuinticPolys
     * @param current_time current time of the internal clock inside the QuinticPolys
     * @return bool
     */
    bool Reset(const std::vector<double> &times, const std::vector<MotionFrame> &points, const double &time_step = 1e-3, const double &current_time = 0, const bool &absolute_time = true);
    
    /**
     * @brief Tick one step of the internal clock
     * 
     */
    void Tick();
    
    /**
     * @brief Calculate the value at current time of internal clock
     * 
     * @return Eigen::Vector3d - the value (position, velocity, acceleration)
     */
    MotionFrame Output();
    
    
    /**
     * @brief Tick one step and then output
     * 
     * @return Eigen::Vector3d
     */
    MotionFrame TickAndOutput();
    
    /**
     * @brief Calculate the value at time t
     * 
     * @param t the time queried
     * @return MotionFrame
     */
    MotionFrame Output(const double &t);
    
    /**
     * @brief Calculate the values at a time series t
     * 
     * @param t the time series queried
     * @return a vector of MotionFrame
     */
    std::vector<MotionFrame> Output(const std::vector<double> &t);

    /**
     * @brief Get the current time of internal clock
     * 
     * @return double
     */
    double GetCurrentTime();
    
    /**
     * @brief Get the time step of internal clock
     * 
     * @return double
     */
    double GetTimeStep();
    
    /**
     * @brief Set the time step of internal clock
     * 
     * @param time_step the time step of internal clock
     */
    void SetTimeStep(const double &time_step);
    
    /**
     * @brief Set the current time of internal clock, default zero
     * 
     * @param current_time current time
     */
    void SetCurrentTime(const double &current_time = 0);

private: 
    std::vector<QuinticPolys> polys_;
    MatrixSlerps rotation_;
    double spinning_vel_;
};

}

#endif
