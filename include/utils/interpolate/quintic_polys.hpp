/*
 * All rights reserved.
 * Copyright (C) 2017 Yangwei You, yangwei.you@foxmail.com
 *
 */


/**
 * @file quintic_polys.hpp
 * @brief File containing the class to construct quintic polynomials.
 *
 * @author Yangwei You
 * @date 12 Dec 2017
 *
 */

#ifndef QUINTIC_POLYS_HPP
#define QUINTIC_POLYS_HPP

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
 * @brief the class to construct a vector of quintic polynomials
 *
 */
class QuinticPolys
{
public:
    QuinticPolys();
    
    /**
     * @brief Class Construct
     * 
     * @param points matrix size n*4, n is the number of points and each row contains the time of the point, its position, velocity, and acceleration.
     * @param time_step time step for each tick of the internal clock inside the QuinticPolys
     * @param current_time current time of the internal clock inside the QuinticPolys
     */
    QuinticPolys(const Eigen::MatrixXd &points, const double &time_step = 1e-3, const double &current_time = 0, const bool &absolute_time = true);
    
    ~QuinticPolys();

    /**
     * @brief Reset the quintic polynomials
     * 
     * @param points matrix size n*3, n is the number of points and each row contains the time of the point, its position, velocity, and acceleration.
     * @param time_step time step for each tick of the internal clock inside the QuinticPolys
     * @param current_time current time of the internal clock inside the QuinticPolys
     * @return bool
     */
    bool Reset(const Eigen::MatrixXd &points, const double &time_step = 1e-3, const double &current_time = 0, const bool &absolute_time = true);
    
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
    Eigen::Vector3d Output();
    
    
    /**
     * @brief Tick one step and then output
     * 
     * @return Eigen::Vector3d
     */
    Eigen::Vector3d TickAndOutput();
    
    /**
     * @brief Calculate the value at time t
     * 
     * @param t the time queried
     * @return Eigen::Vector3d - the value (position, velocity, acceleration)
     */
    Eigen::Vector3d Output(const double &t);
    
    /**
     * @brief Calculate the values at a time series t
     * 
     * @param t the time series queried
     * @return Eigen::MatrixXd - the value series with size 3*t.size()
     */
    Eigen::MatrixXd Output(const Eigen::VectorXd &t);

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
    Eigen::MatrixXd GetCoef(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const double &duration);
    double current_time_;
    double time_step_;
    std::vector<Eigen::MatrixXd> coefs_;
    Eigen::VectorXd times_;
};

}

#endif
