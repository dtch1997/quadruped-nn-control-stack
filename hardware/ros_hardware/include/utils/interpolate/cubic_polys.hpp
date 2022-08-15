/*
 * All rights reserved.
 * Copyright (C) 2017 Yangwei You, yangwei.you@foxmail.com
 *
 */


/**
 * @file cubic_polys.hpp
 * @brief File containing the class to construct cubic polynomials.
 *
 * @author Yangwei You
 * @date 27 Nov 2017
 *
 */

#ifndef CUBIC_POLYS_HPP
#define CUBIC_POLYS_HPP

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
 * @brief the class to construct a vector of cubic polynomials
 *
 */
class CubicPolys
{
public:
    CubicPolys();
    
    /**
     * @brief Class Construct
     * 
     * @param points matrix size n*3, n is the number of points and each row contains the time of the point, its position and velocity.
     * @param time_step time step for each tick of the internal clock inside the CubicPolys
     * @param current_time current time of the internal clock inside the CubicPolys
     * @param absolute_time the time for each point is absolute instead of representing each period
     */
    CubicPolys(const Eigen::MatrixXd &points, const double &time_step = 1e-3, const double &current_time = 0, const bool &absolute_time = true);
    
    ~CubicPolys();

    /**
     * @brief Reset the cubic polynomials
     * 
     * @param points matrix size n*3, n is the number of points and each row contains the time of the point, its position and velocity.
     * @param time_step time step for each tick of the internal clock inside the CubicPolys
     * @param current_time current time of the internal clock inside the CubicPolys
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
    Eigen::MatrixXd GetCoef(const double &y0, const double &dy0, const double &yf, const double &dyf, const double &t);
    double current_time_;
    double time_step_;
    std::vector<Eigen::MatrixXd> coefs_;
    Eigen::VectorXd times_;
};

}

#endif
