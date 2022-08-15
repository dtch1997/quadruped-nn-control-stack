/*
 * All rights reserved.
 * Copyright (C) 2017 Yangwei You, yangwei.you@foxmail.com
 *
 */


/**
 * @file matrix_slerps.hpp
 * @brief File containing the class to do rotation slerp with matrix.
 *
 * @author Yangwei You
 * @date 25 Jan 2017
 *
 */

#ifndef MATRIX_SLERPS_HPP
#define MATRIX_SLERPS_HPP

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>

/**
 * @brief namespace for the modules of legged robot
 *
 */
namespace legged_robot
{

/**
 * @brief the class to do rotation slerp with matrix.
 *
 */
class MatrixSlerps
{
public:
    MatrixSlerps();
    
    /**
     * @brief Class Construct
     * 
     * @param times time series for each oritation
     * @param points a series of oritations.
     * @param time_step time step for each tick of the internal clock inside the MatrixSlerps
     * @param current_time current time of the internal clock inside the MatrixSlerps
     * @param absolute_time the time series is absolute or not
     */
    MatrixSlerps(const std::vector<double> times, const std::vector<Eigen::Matrix3d> &oritations, const double &time_step = 1e-3, const double &current_time = 0, const bool &absolute_time = true);
    
    ~MatrixSlerps();
    
    /**
     * @brief Reset
     * 
     * @param times time series for each oritation
     * @param points a series of oritations.
     * @param time_step time step for each tick of the internal clock inside the MatrixSlerps
     * @param current_time current time of the internal clock inside the MatrixSlerps
     * @param absolute_time the time series is absolute or not
     */
    bool Reset(const std::vector<double> times, const std::vector<Eigen::Matrix3d> &oritations, const double &time_step = 1e-3, const double &current_time = 0, const bool &absolute_time = true);
    
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
    Eigen::Matrix3d Output();
    
    
    /**
     * @brief Tick one step and then output
     * 
     * @return Eigen::Vector3d
     */
    Eigen::Matrix3d TickAndOutput();
    
    /**
     * @brief Calculate the value at time t
     * 
     * @param t the time queried
     * @return Eigen::Vector3d - the value (position, velocity, acceleration)
     */
    Eigen::Matrix3d Output(const double &t);
    
    /**
     * @brief Calculate the values at a time series t
     * 
     * @param t the time series queried
     * @return Eigen::MatrixXd - the value series with size 3*t.size()
     */
    std::vector<Eigen::Matrix3d> Output(const std::vector<double> &t);

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
    double current_time_;
    double time_step_;
    std::vector<Eigen::Quaterniond> coefs_;
    std::vector<double> times_;
};

}

#endif
