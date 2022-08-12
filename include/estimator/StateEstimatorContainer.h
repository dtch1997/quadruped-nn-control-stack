#ifndef PROJECT_STATEESTIMATOR_H
#define PROJECT_STATEESTIMATOR_H

#include "types/imu_types.h"
#include "utils/utilities_print.h"
#include <yaml-cpp/yaml.h>

template<typename T>
struct StateEstimate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec4<T> contactEstimate;
    Vec3<T> position;
    Vec3<T> vBody;
    Quat<T> orientation;
    Vec3<T> omegaBody;
    RotMat<T> rBody;
    Vec3<T> rpy;

    Vec3<T> omegaWorld;
    Vec3<T> vWorld;
    Vec3<T> aBody, aWorld;

    Mat34<T> footPos_body;
    Mat34<T> footVel_body;
    T phaseValue;

};

typedef struct
{
  float quaternion[4];               // quaternion, normalized, (w,x,y,z)
  float gyroscope[3];                // angular velocity （unit: rad/s)
  float accelerometer[3];            // m/(s2)
  float rpy[3];                      // euler angle（unit: rad)
  int8_t temperature;
} IMU;                               // when under accelerated motion, the attitude of the robot calculated by IMU will drift.

template<typename T>
struct StateEstimatorData {
    StateEstimate<T> *result;  // where to write the output to
    IMU *vectorNavData;
    Vec4<T> *contactPhase;
};

template<typename T>
class GenericEstimator {
public:
    virtual void run() = 0;
    virtual void setup() = 0;
    virtual void print_estimate() = 0;
    void setData(StateEstimatorData<T> data) { _stateEstimatorData = data; }
    virtual void setJointState(Eigen::VectorXf jPos,Eigen::VectorXf jVel) = 0;
    virtual ~GenericEstimator() = default;
    StateEstimatorData<T> _stateEstimatorData;
};

template<typename T>
class StateEstimatorContainer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * Construct a new state estimator container
     */
    StateEstimatorContainer(IMU *vectorNavData, StateEstimate<T> *stateEstimate) {
      _data.vectorNavData = vectorNavData;
      _data.result = stateEstimate;
      _phase = Vec4<T>::Zero();
      _data.contactPhase = &_phase;
      //_data.cheaterState = cheaterState;
    }

    // Run all estimator
    void run() {
      for (auto estimator: _estimators) estimator->run();
    }

    void print_estimate() {
      for (auto estimator: _estimators) estimator->print_estimate();
    }

    const StateEstimate<T> &getResult() { return *_data.result; }
    const IMU &getVectorNavData() { return *_data.vectorNavData; }
    StateEstimate<T> *getResultHandle() { return _data.result; }

    void setContactPhase(Vec4<T> &phase) {
      *_data.contactPhase = phase;
    }

    template<typename EstimatorToAdd>
    void addEstimator() {
      auto *estimator = new EstimatorToAdd();
      estimator->setData(_data);
      estimator->setup();
      _estimators.push_back(estimator);
    }

    template<typename EstimatorToAdd>
    void addEstimator(YAML::Node yml) {
      auto *estimator = new EstimatorToAdd(yml);
      estimator->setData(_data);
      estimator->setup();
      _estimators.push_back(estimator);
    }

    template<typename EstimatorToRemove>
    void removeEstimator() {
      int nRemoved = 0;
      _estimators.erase(
              std::remove_if(_estimators.begin(), _estimators.end(),
                             [&nRemoved](GenericEstimator<T> *e) {
                                 if (dynamic_cast<EstimatorToRemove *>(e)) {
                                   delete e;
                                   nRemoved++;
                                   return true;
                                 } else {
                                   return false;
                                 }
                             }),
              _estimators.end());
    }

    void removeAllEstimators() {
      for (auto estimator: _estimators) delete estimator;
      _estimators.clear();
    }
    void resetAllEstimators() {
      for (auto estimator: _estimators) estimator->setup();
      _phase = Vec4<T>::Zero();
    }



    void setJointState(Eigen::VectorXf q,Eigen::VectorXf dq){
      for (auto estimator: _estimators) estimator->setJointState(q,dq);
    }

    ~StateEstimatorContainer() {
      for (auto estimator: _estimators) delete estimator;
    }


private:
    StateEstimatorData<T> _data;
    std::vector<GenericEstimator<T> *> _estimators;
    Vec4<T> _phase;
};

#endif  // PROJECT_STATEESTIMATOR_H
