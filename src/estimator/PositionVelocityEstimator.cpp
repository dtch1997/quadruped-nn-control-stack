/*! @file PositionVelocityEstimator.cpp
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#include "estimator/PositionVelocityEstimator.h"

/*!
 * Initialize the state estimator
 */
template<typename T>
void LinearKFPositionVelocityEstimator<T>::setup() {

  dt = 1.0 / _yaml["control_frequency"].as<T>();

  const std::vector<T> hipPos_yml = _yaml["absolute_hip_pos"].as<std::vector<T>>();
  assert((int)hipPos_yml.size() == 3);
  for(int i(0);i<3;i++) hipPos_abs[i] = hipPos_yml[i];
  std::cout<<"Read Hip: "<< hipPos_abs.transpose() << std::endl;

  l1 = _yaml["hip_link_length"].as<T>();
  l2 = _yaml["thigh_link_length"].as<T>();
  l3 = _yaml["calf_link_length"].as<T>();
  l4 = _yaml["calf_offset_y"].as<T>();

  process_noise_pimu = _yaml["imu_process_noise_position"].as<T>();
  process_noise_vimu = _yaml["imu_process_noise_velocity"].as<T>();
  process_noise_pfoot = _yaml["foot_process_noise_position"].as<T>();
  sensor_noise_pimu_rel_foot = _yaml["foot_sensor_noise_position"].as<T>();
  sensor_noise_vimu_rel_foot = _yaml["foot_sensor_noise_velocity"].as<T>();
  sensor_noise_zfoot = _yaml["foot_height_sensor_noise"].as<T>();


  gaitFreq = _yaml["step_freq"].as<T>();
  dutyCycle = _yaml["duty_cycle"].as<T>();


  _xhat.setZero();
  _ps.setZero();
  _vs.setZero();
  _A.setZero();
  _A.block(0, 0, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(3, 3, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(6, 6, 12, 12) = Eigen::Matrix<T, 12, 12>::Identity();
  _B.setZero();
  _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
  C1 << Eigen::Matrix<T, 3, 3>::Identity(), Eigen::Matrix<T, 3, 3>::Zero();
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
  C2 << Eigen::Matrix<T, 3, 3>::Zero(), Eigen::Matrix<T, 3, 3>::Identity();
  _C.setZero();
  _C.block(0, 0, 3, 6) = C1;
  _C.block(3, 0, 3, 6) = C1;
  _C.block(6, 0, 3, 6) = C1;
  _C.block(9, 0, 3, 6) = C1;
  _C.block(0, 6, 12, 12) = T(-1) * Eigen::Matrix<T, 12, 12>::Identity();
  _C.block(12, 0, 3, 6) = C2;
  _C.block(15, 0, 3, 6) = C2;
  _C.block(18, 0, 3, 6) = C2;
  _C.block(21, 0, 3, 6) = C2;
  _C(27, 17) = T(1);
  _C(26, 14) = T(1);
  _C(25, 11) = T(1);
  _C(24, 8) = T(1);
  _P.setIdentity();
  _P = T(100) * _P;
  _Q0.setIdentity();
  _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
  _Q0.block(3, 3, 3, 3) =
          (dt * 9.8f / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
  _Q0.block(6, 6, 12, 12) = dt * Eigen::Matrix<T, 12, 12>::Identity();
  _R0.setIdentity();
  this->_stateEstimatorData.result->position.setZero();
  this->_stateEstimatorData.result->vWorld.setZero();
  this->_stateEstimatorData.result->vBody.setZero();
  this->_stateEstimatorData.result->footPos_body.setZero();
  this->_stateEstimatorData.result->footVel_body.setZero();
  this->_stateEstimatorData.result->phaseValue = 0.;
}

template<typename T>
LinearKFPositionVelocityEstimator<T>::LinearKFPositionVelocityEstimator(YAML::Node &yaml):
_yaml(yaml)
{}

/*!
 * Run state estimator
 */
template<typename T>
void LinearKFPositionVelocityEstimator<T>::run() {

  Eigen::Matrix<T, 18, 18> Q = Eigen::Matrix<T, 18, 18>::Identity();
  Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
  Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
  Q.block(6, 6, 12, 12) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot;

  Eigen::Matrix<T, 28, 28> R = Eigen::Matrix<T, 28, 28>::Identity();
  R.block(0, 0, 12, 12) = _R0.block(0, 0, 12, 12) * sensor_noise_pimu_rel_foot;
  R.block(12, 12, 12, 12) =
          _R0.block(12, 12, 12, 12) * sensor_noise_vimu_rel_foot;
  R.block(24, 24, 4, 4) = _R0.block(24, 24, 4, 4) * sensor_noise_zfoot;

  int qindex = 0;
  int rindex1 = 0;
  int rindex2 = 0;
  int rindex3 = 0;

  Vec3<T> g(0, 0, T(-9.81));
  Mat3<T> Rbod = this->_stateEstimatorData.result->rBody.transpose();
  // in old code, Rbod * se_acc + g
  Vec3<T> a = this->_stateEstimatorData.result->aWorld + g;
  // std::cout << "A WORLD\n" << a << "\n";
  Vec4<T> pzs = Vec4<T>::Zero();
  Vec4<T> trusts = Vec4<T>::Zero();
  Vec3<T> p0, v0;
  p0 << _xhat[0], _xhat[1], _xhat[2];
  v0 << _xhat[3], _xhat[4], _xhat[5];

  updateDesiredContact();
  for (int i = 0; i < 4; i++) {
    int i_old = i;
//    i_old = i%2==0?i+1:i-1;
//    std::cout<<"loop:"<<i<<"\told:"<<i_old<<std::endl;
    int i1 = 3 * i_old;
    computeLegJacobianFootState(i_old);
    p_rel = p_rel + getHipLocation(i_old);
    this->_stateEstimatorData.result->footPos_body.col(i_old) = p_rel;
    this->_stateEstimatorData.result->footVel_body.col(i_old) = dp_rel;
    p_f = Rbod * p_rel;
    dp_f = Rbod * (this->_stateEstimatorData.result->omegaBody.cross(p_rel) + dp_rel);

    qindex = 6 + i1;
    rindex1 = i1;
    rindex2 = 12 + i1;
    rindex3 = 24 + i_old;

    T trust = T(1);
    T phase = fmin(this->_stateEstimatorData.result->contactEstimate(i), T(1));
    T trust_window = T(0.2);

    if (phase < trust_window) {
      trust = phase / trust_window;
    } else if (phase > (T(1) - trust_window)) {
      trust = (T(1) - phase) / trust_window;
    }
    //T high_suspect_number(1000);
    T high_suspect_number(100);

    // printf("Trust %d: %.3f\n", i, trust);
    Q.block(qindex, qindex, 3, 3) = (T(1) + (T(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
    R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
    R.block(rindex2, rindex2, 3, 3) = (T(1) + (T(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
    R(rindex3, rindex3) = (T(1) + (T(1) - trust) * high_suspect_number) * R(rindex3, rindex3);

    trusts(i_old) = trust;

    _ps.segment(i1, 3) = -p_f;
    _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
    pzs(i_old) = (1.0f - trust) * (p0(2) + p_f(2));
  }

  Eigen::Matrix<T, 28, 1> y;
  y << _ps, _vs, pzs;
  _xhat = _A * _xhat + _B * a;
  Eigen::Matrix<T, 18, 18> At = _A.transpose();
  Eigen::Matrix<T, 18, 18> Pm = _A * _P * At + Q;
  Eigen::Matrix<T, 18, 28> Ct = _C.transpose();
  Eigen::Matrix<T, 28, 1> yModel = _C * _xhat;
  Eigen::Matrix<T, 28, 1> ey = y - yModel;
  Eigen::Matrix<T, 28, 28> S = _C * Pm * Ct + R;

  Eigen::Matrix<T, 28, 1> S_ey = S.lu().solve(ey);
  _xhat += Pm * Ct * S_ey;

  Eigen::Matrix<T, 28, 18> S_C = S.lu().solve(_C);
  _P = (Eigen::Matrix<T, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;

  Eigen::Matrix<T, 18, 18> Pt = _P.transpose();
  _P = (_P + Pt) / T(2);

  if (_P.block(0, 0, 2, 2).determinant() > T(0.000001)) {
    _P.block(0, 2, 2, 16).setZero();
    _P.block(2, 0, 16, 2).setZero();
    _P.block(0, 0, 2, 2) /= T(10);
  }

  this->_stateEstimatorData.result->position.setZero();
  this->_stateEstimatorData.result->vWorld.setZero();
  this->_stateEstimatorData.result->vBody.setZero();
  this->_stateEstimatorData.result->position = _xhat.block(0, 0, 3, 1);
  this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1);
  this->_stateEstimatorData.result->vBody =
          this->_stateEstimatorData.result->rBody *
          this->_stateEstimatorData.result->vWorld;
}

template<typename T>
void LinearKFPositionVelocityEstimator<T>::print_estimate() {
  printf("\n \t\tPOSITION-VELOCITY ESTIMATE\n");
  pretty_print(this->_stateEstimatorData.result->vBody, std::cout, "vBody"); //orientation
  pretty_print(this->_stateEstimatorData.result->vWorld, std::cout, "vWorld"); //rpy
  pretty_print(this->_stateEstimatorData.result->position, std::cout, "position"); //omegabody
  printf("\n \t\tPHASE & CONTACT");
  std::cout<<"phase = "<<this->_stateEstimatorData.result->phaseValue<<std::endl;
  std::cout<<"contact = "<<this->_stateEstimatorData.result->contactEstimate.transpose()<<std::endl;
}

template
class LinearKFPositionVelocityEstimator<float>;

template
class LinearKFPositionVelocityEstimator<double>;

template<typename T>
void LinearKFPositionVelocityEstimator<T>::setJointState(Eigen::VectorXf jPos,Eigen::VectorXf jVel) {
  _jntPos = jPos.tail(12).cast<T>();
  _jntVel = jVel.tail(12).cast<T>();
}

template<typename T>
void LinearKFPositionVelocityEstimator<T>::updateDesiredContact() {

  this->_stateEstimatorData.result->phaseValue += dt * gaitFreq;
  if (this->_stateEstimatorData.result->phaseValue > 1.0) this->_stateEstimatorData.result->phaseValue -= 1.0;
  Vec4<T> gaitOffset;
  gaitOffset << 0.5, 0., 0., 0.5; // Left First Convention
  for (int i = 0; i < 4; i++) {
    T footPhase = fmod(this->_stateEstimatorData.result->phaseValue + gaitOffset[i], 1.0);
    if (footPhase < dutyCycle) { this->_stateEstimatorData.result->contactEstimate[i] = 0.; }
    else { this->_stateEstimatorData.result->contactEstimate[i] = (footPhase - dutyCycle) / (1.0 - dutyCycle); }
  }
}

template<typename T>
void LinearKFPositionVelocityEstimator<T>::computeLegJacobianFootState(int leg) {


  Vec4<T> sideSigns;
  sideSigns << 1, -1, 1, -1; // Right First
  T sideSign = sideSigns[leg];

  int lIdx = leg;
//  lIdx = leg % 2 == 0 ? leg + 1 : leg - 1;
  _q = _jntPos.segment(lIdx * 3, 3);
  _dq = _jntVel.segment(lIdx * 3, 3);
  _q[1] = -_q[1];
  _q[2] = -_q[2];
  _dq[1] = -_dq[1];
  _dq[2] = -_dq[2];
  T s1 = std::sin(_q[0]);
  T s2 = std::sin(_q[1]);
  T s3 = std::sin(_q[2]);

  T c1 = std::cos(_q[0]);
  T c2 = std::cos(_q[1]);
  T c3 = std::cos(_q[2]);

  T c23 = c2 * c3 - s2 * s3;
  T s23 = s2 * c3 + c2 * s3;

  J(0, 0) = 0;
  J(0, 1) = l3 * c23 + l2 * c2;
  J(0, 2) = l3 * c23;
  J(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * sideSign * s1;
  J(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
  J(1, 2) = -l3 * s1 * s23;
  J(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * sideSign * c1;
  J(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
  J(2, 2) = l3 * c1 * s23;

  p_rel[0] = l3 * s23 + l2 * s2;
  p_rel[1] = (l1 + l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
  p_rel[2] = (l1 + l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;

  dp_rel = J * _dq;

//  std::cout << "footPos["<<leg<<"]: "<< p_rel.transpose() <<std::endl;
//  std::cout << "footVel["<<leg<<"]: "<< dp_rel.transpose() <<std::endl;
}

template<typename T>
Vec3<T> LinearKFPositionVelocityEstimator<T>::getHipLocation(int leg) {
  assert(leg >= 0 && leg < 4); // FR FL HR HL
  Vec3<T> hipSign((leg == 0 || leg == 1) ? -1 : 1, (leg == 1 || leg == 3) ? -1 : 1, 1);
  return hipPos_abs.cwiseProduct(hipSign);
}