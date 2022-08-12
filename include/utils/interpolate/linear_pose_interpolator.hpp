/*
 * All rights reserved.
 * Copyright (C) 2017 Yangwei You, yangwei.you@foxmail.com
 *
 */


/**
 * @file linear_pose_interpolator.hpp
 * @brief File containing the class to construct a linear interpolator for pose.
 *
 * @author Yangwei You
 * @date 27 August 2017
 *
 */

#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "matrix_slerps.hpp"

/**
 * @brief namespace for the modules of legged robot
 *
 */
namespace legged_robot
{

/**
 * @brief the class to construct a linear interpolator for pose.
 *
 */
class LinearPoseInterpolator
{
public:
    LinearPoseInterpolator();
    
    /**
     * @brief Class Construct
     * 
     * @param times a vector of serial time.
     * @param points a vector of serial pose.
     * @param time_step time step for each tick of the internal clock inside the QuinticPolys
     * @param current_time current time of the internal clock inside the QuinticPolys
     * @param absolute_time the input time serial is absolute or relative
     */
    LinearPoseInterpolator(const std::vector<double> &times, const std::vector<Eigen::Affine3d> &points, const double &time_step = 1e-3, const double &current_time = 0, const bool &absolute_time = true);
    
    ~LinearPoseInterpolator();

    /**
     * @brief Reset the quintic polynomials pose
     * 
     * @param points a vector of serial MotionFrame.
     * @param time_step time step for each tick of the internal clock inside the QuinticPolys
     * @param current_time current time of the internal clock inside the QuinticPolys
     * @return bool
     */
    bool Reset(const std::vector<double> &times, const std::vector<Eigen::Affine3d> &points, const double &time_step = 1e-3, const double &current_time = 0, const bool &absolute_time = true);
    
    /**
     * @brief Tick one step of the internal clock
     * 
     */
    void Tick();
    
    /**
     * @brief Calculate the value at current time of internal clock
     * 
     * @return Eigen::Affine3d
     */
    Eigen::Affine3d Output();
    
    /**
     * @brief Tick one step and then output
     * 
     * @return Eigen::Affine3d
     */
    Eigen::Affine3d TickAndOutput();
    
    /**
     * @brief Calculate the value at time t
     * 
     * @param t the time queried
     * @return Eigen::Affine3d
     */
    Eigen::Affine3d Output(const double &t);
    
    /**
     * @brief Calculate the values at a time series t
     * 
     * @param t the time series queried
     * @return a vector of Eigen::Affine3d
     */
    std::vector<Eigen::Affine3d> Output(const std::vector<double> &t);

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
    
    /**
     * @brief Check if the whole trajectory has been done
     * 
     * @return bool
     */
    bool IsDone();

private: 
    std::vector<Eigen::Affine3d> poses_;
    std::vector<double> times_;
    double current_time_;
    double step_time_;
};

}