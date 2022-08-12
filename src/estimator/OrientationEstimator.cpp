/*! @file OrientationEstimator.cpp
 *  @brief All Orientation Estimation Algorithms
 *
 *  This file will contain all orientation algorithms.
 *  Orientation estimators should compute:
 *  - orientation: a quaternion representing orientation
 *  - rBody: coordinate transformation matrix (satisfies vBody = Rbody * vWorld)
 *  - omegaBody: angular velocity in body frame
 *  - omegaWorld: angular velocity in world frame
 *  - rpy: roll pitch yaw
 */

#include "estimator/OrientationEstimator.h"

template <typename T>
void VectorNavOrientationEstimator<T>::setup() {
  _b_first_visit = true;
}

/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 */
template <typename T>
void VectorNavOrientationEstimator<T>::run() {
  for(int i(0);i<4;i++) {
    this->_stateEstimatorData.result->orientation[i] = this->_stateEstimatorData.vectorNavData->quaternion[i];
  }
  if(_b_first_visit){
    Vec3<T> rpy_ini = ori::quatToRPY(this->_stateEstimatorData.result->orientation);
    rpy_ini.head(2).setZero();
    _ori_ini_inv = ori::rpyToQuat(-rpy_ini);  //CHECK Why ori:: is not included in Cheetah Software
    _b_first_visit = false;
  }
  this->_stateEstimatorData.result->orientation = 
    ori::quatProduct(_ori_ini_inv, this->_stateEstimatorData.result->orientation);

  this->_stateEstimatorData.result->rpy =
      ori::quatToRPY(this->_stateEstimatorData.result->orientation);

  this->_stateEstimatorData.result->rBody = ori::quaternionToRotationMatrix(
      this->_stateEstimatorData.result->orientation);

  for(int i(0);i<3;i++) {
    this->_stateEstimatorData.result->omegaBody[i] = this->_stateEstimatorData.vectorNavData->gyroscope[i];
    this->_stateEstimatorData.result->aBody[i] = this->_stateEstimatorData.vectorNavData->accelerometer[i];
  }

  this->_stateEstimatorData.result->omegaWorld =
          this->_stateEstimatorData.result->rBody.transpose() *
          this->_stateEstimatorData.result->omegaBody;

  this->_stateEstimatorData.result->aWorld =
      this->_stateEstimatorData.result->rBody.transpose() *
      this->_stateEstimatorData.result->aBody;

}

template <typename T>
void VectorNavOrientationEstimator<T>::print_estimate() 
{
    printf("\n \t\tORIENTATION ESTIMATE\n");
    pretty_print(this->_stateEstimatorData.result->orientation, std::cout, "Orientation"); //orientation
    pretty_print(this->_stateEstimatorData.result->rpy, std::cout, "rpy"); //rpy
    pretty_print(this->_stateEstimatorData.result->omegaBody, std::cout, "omegabody"); //omegabody
    pretty_print(this->_stateEstimatorData.result->aBody, std::cout, "aBody");//abody 
    pretty_print(this->_stateEstimatorData.result->aWorld, std::cout, "aWorld");//aworld
}

// template class CheaterOrientationEstimator<float>;
// template class CheaterOrientationEstimator<double>;

template class VectorNavOrientationEstimator<float>;
template class VectorNavOrientationEstimator<double>;

template<typename T>
void VectorNavOrientationEstimator<T>::setJointState(Eigen::VectorXf jPos, Eigen::VectorXf jVel) {}
