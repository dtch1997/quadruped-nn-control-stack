#ifndef CUSTOM_TYPES_H
#define CUSTOM_TYPES_H
 
#include <Eigen/Dense>
#include <iostream>
#include <vector>

namespace CustomTypes {
    typedef struct State {
        Eigen::VectorXf basePosition; 
        Eigen::VectorXf baseVelocity;
        Eigen::MatrixXf baseRotMat;
        Eigen::VectorXf baseRpyRate;
        Eigen::VectorXf motorPosition;
        Eigen::VectorXf motorVelocity;
        Eigen::VectorXf baseVelocity_w;
        Eigen::VectorXf baseRpyRate_w;
        float phaseValue;
    } State;

    typedef struct Action{
        Eigen::VectorXf motorPosition;
        Eigen::VectorXf motorVelocity;
        Eigen::VectorXf kP;
        Eigen::VectorXf kD;
    } Action;

    inline State zeroState() {
      struct State ret;
      ret.motorPosition.setZero(12);
      ret.motorVelocity.setZero(12);
      ret.basePosition.setZero(3);
      ret.baseVelocity.setZero(3);
      ret.baseRotMat.setZero(3,3);
      ret.baseRpyRate.setZero(3);
      ret.phaseValue = 0.;
      return ret;
    }

    inline Action zeroAction() {
      struct Action ret;
      ret.motorPosition.setZero(12);
      ret.motorVelocity.setZero(12);
      ret.kP.setZero(12);
      ret.kD.setZero(12);
      return ret;
    }
};

#endif
