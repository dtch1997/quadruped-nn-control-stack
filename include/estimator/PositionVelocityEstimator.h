/*! @file PositionVelocityEstimator.h
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#ifndef PROJECT_POSITIONVELOCITYESTIMATOR_H
#define PROJECT_POSITIONVELOCITYESTIMATOR_H

#include "estimator/StateEstimatorContainer.h"
#include "types/cpp_types.h"
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <vector>

/*!
 * Position and velocity estimator based on a Kalman Filter.
 * This is the algorithm used in Mini Cheetah and Cheetah 3.
 */
template<typename T>
class LinearKFPositionVelocityEstimator : public GenericEstimator<T> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinearKFPositionVelocityEstimator(YAML::Node &yaml);

    virtual void run();

    virtual void setup();

    virtual void print_estimate();

    virtual void setJointState(Eigen::VectorXf jPos,Eigen::VectorXf jVel);

private:
    YAML::Node _yaml;

    Eigen::Matrix<T, 18, 1> _xhat;
    Eigen::Matrix<T, 12, 1> _ps;
    Eigen::Matrix<T, 12, 1> _vs;
    Eigen::Matrix<T, 18, 18> _A;
    Eigen::Matrix<T, 18, 18> _Q0;
    Eigen::Matrix<T, 18, 18> _P;
    Eigen::Matrix<T, 28, 28> _R0;
    Eigen::Matrix<T, 18, 3> _B;
    Eigen::Matrix<T, 28, 18> _C;

    Vec3<T> ph;  // hip positions relative to CoM
    Vec3<T> p_rel;
    Vec3<T> dp_rel;
    Vec3<T> p_f;
    Vec3<T> dp_f;

    Vec12<T> _jntPos, _jntVel;

    Vec3<T> _q, _dq;
    Mat3<T> J;



    Vec3<T> hipPos_abs;
    T l1; //_abadLinkLength;
    T l2; //_hipLinkLength;
    T l3; //_kneeLinkLength;
    T l4; //_kneeLinkY_offset


    T process_noise_pimu = 0.;
    T process_noise_vimu = 0.;
    T process_noise_pfoot = 0.;
    T sensor_noise_pimu_rel_foot = 0.;
    T sensor_noise_vimu_rel_foot = 0.;
    T sensor_noise_zfoot = 0.;


    T gaitFreq = 2;
    T dutyCycle = 0.5;
    T dt = 1e-3;
    void computeLegJacobianFootState(int leg);

    Vec3<T> getHipLocation(int leg);

    void updateDesiredContact();
};


#endif  // PROJECT_POSITIONVELOCITYESTIMATOR_H
