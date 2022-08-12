
#ifndef PROJECT_IMUTYPES_H
#define PROJECT_IMUTYPES_H

#include "cpp_types.h"

/*!
 * Mini Cheetah's IMU
 */
struct VectorNavData {
  Vec3<float> accelerometer;
  Vec3<float> gyro;
  Quat<float> quat; // convention: WXYZ
};

/*!
 * "Cheater" state sent to the robot from simulator
 */
template <typename T>
struct CheaterState {
  Quat<T> orientation;
  Vec3<T> position;
  Vec3<T> omegaBody;
  Vec3<T> vBody;
  Vec3<T> acceleration;
};

#endif  // PROJECT_IMUTYPES_H
