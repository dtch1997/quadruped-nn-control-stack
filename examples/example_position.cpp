//#define DEBUG

#include <thread>
#include <csignal>
#include <yaml-cpp/yaml.h>
#include "controller/Controller.h"
#include "controller/SequentialController.h"
#include "controller/ResetController.h"
#include "controller/ZeroOrderHolder.h"
#include "controller/NeuralController.h"
#include "controller/FixedPosController.h"
#include "estimator/StateEstimatorContainer.h"
#include "estimator/PositionVelocityEstimator.h"
#include "estimator/OrientationEstimator.h"
#include "utils/orientation_tools.h"
#include "hardware_interface.h"

double jointLinearInterpolation(double initPos, double targetPos, double rate) {
  double p;
  rate = std::min(std::max(rate, 0.0), 1.0);
  p = initPos * (1 - rate) + targetPos * rate;
  return p;
}

class Custom : public HardwareInterface {

public:
    explicit Custom(const YAML::Node &yaml)
            : _yaml(yaml) {

      _q.setZero(12);
      _qd.setZero(12);
      angularVelTarg.setZero();
      linearVelTarg.setZero();
      _upperPosLimit.setZero(12);
      _lowerPosLimit.setZero(12);
      auto upHAA = _yaml["HAA_upper_limit"].as<float>();
      auto loHAA = _yaml["HAA_lower_limit"].as<float>();
      auto upHFE = _yaml["HFE_upper_limit"].as<float>();
      auto loHFE = _yaml["HFE_lower_limit"].as<float>();
      auto upKFE = _yaml["KFE_upper_limit"].as<float>();
      auto loKFE = _yaml["KFE_lower_limit"].as<float>();
      _upperPosLimit << upHAA, upHFE, upKFE, upHAA, upHFE, upKFE, upHAA, upHFE, upKFE, upHAA, upHFE, upKFE;
      _lowerPosLimit << loHAA, loHFE, loKFE, loHAA, loHFE, loKFE, loHAA, loHFE, loKFE, loHAA, loHFE, loKFE;

      auto kMaxTimesteps = _yaml["standup_timesteps"].as<int>();
      auto waitTimesteps = _yaml["standby_timesteps"].as<int>();
      controlFreq = _yaml["control_frequency"].as<float>();
      std::string ConfigDir = _yaml["reset_pose_path"].as<std::string>();
      const std::vector<float> VelVec = _yaml["default_velocity_target"].as<std::vector<float>>();
      for (int i = 0; i < 3; i++)linearVelTarg[i] = VelVec[i];

      _stateEstimator = new StateEstimatorContainer<float>(&(data.imu), &_stateEstimate);
      _stateEstimator->removeAllEstimators();
      _stateEstimator->addEstimator<VectorNavOrientationEstimator<float >>();
      _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float >>(_yaml);
      _stateEstimate.phaseValue = 0.;

      Eigen::VectorXd finalMotorPosition(12), midMotorPosition(12);
      finalMotorPosition = MLP_Actor::readCSV(ConfigDir + "/final_motor_position.csv", 12, 1).col(0);
      midMotorPosition = MLP_Actor::readCSV(ConfigDir + "/mid_motor_position.csv", 12, 1).col(0);
      if(_yaml["strech_only"].as<bool>()){
        std::cout << "Test Stance Pos Only...Gonna wait for an hour.\n";
        waitTimesteps = int(3600 * controlFreq);
      }
      _actionDataset.setZero(1,12);
      _replayFlag = _yaml["action_replay"].as<bool>();
      actionDatasetLength =  _yaml["replay_length"].as<int>();
      if(_replayFlag){
        std::cout << "Going to replay action from dataset..\n";
        _actionDataset = MLP_Actor::readCSV(_yaml["action_record_file"].as<std::string>(),actionDatasetLength, 12).cast<float>();
        actionRecordIdx = 0;
        holdCount = 0;
      }

      hold_frames = (int) (controlFreq / _yaml["policy_frequency"].as<float>());
      std::vector<std::shared_ptr<Controller>> _base_controllers;
      _base_controllers.push_back(
              std::make_shared<ResetController>(finalMotorPosition.cast<float>(), midMotorPosition.cast<float>(),
                                                kMaxTimesteps, 1.0f / controlFreq)
      );
      _base_controllers.push_back(
              std::make_shared<FixedPosController>(finalMotorPosition.cast<float>(), waitTimesteps)
      );
      _base_controllers.push_back(
              std::make_shared<ZeroOrderHolder>(std::make_shared<NeuralController>(
                                                        _yaml["save_path"].as<std::string>(),
                                                        _yaml["weight_path"].as<std::string>()),
                                                hold_frames)
      );
      _controller = new SequentialController(_base_controllers);
      nnCtrlIdx = 2;
    }

    ~Custom() override = default;;

    long long count = 0;

private:
    void RobotControl() override {

      DBG_INFO("Start Control Loop.");
      getMotorStates(_q, _qd);
      _stateEstimator->setJointState(_q, _qd);
      DBG_INFO("Run Estimator");

      _stateEstimator->run();
      DBG_INFO("Assign Time Var");
      _controller->sendTime(systemTime);
      DBG_INFO("YAML Vel Target: " << linearVelTarg.transpose());
      _controller->setBaseVelTarget(linearVelTarg, angularVelTarg);
      DBG_INFO("Get Action");

      if (!phaseInitialized && nnCtrlIdx == _controller->getCurrIdx()) {
        _stateEstimate.phaseValue = 0.;
        phaseInitialized = true;
      }

      robotState = getStateEstimation();

      CustomTypes::Action robotAction = CustomTypes::zeroAction();
      if(systemTime>0.02) robotAction = _controller->getControlAction(robotState);

      if (_controller->getCurrIdx() > 1 && (robotState.baseRotMat(2, 2) < 0.4 || robotState.basePosition[2] < 0.15 ||
                                            robotState.basePosition[2] > 0.5)) {
        std::cout << "Pose Overflow!" << robotState.baseRotMat.row(2) << "~" << robotState.basePosition.transpose()
                  << " Inevitable Falling. Pls Reset Robot." << std::endl;
        printState(robotState);
        _stateEstimator->print_estimate();
        _controller->reset();
        _stateEstimator->resetAllEstimators();
        phaseInitialized = false;
      }

      if(_replayFlag && nnCtrlIdx == _controller->getCurrIdx()){
        robotAction.motorPosition = _actionDataset.row(actionRecordIdx);
        holdCount += 1;
        if(holdCount == hold_frames){
          holdCount = 0;
          actionRecordIdx += 1;
        }
        if (actionRecordIdx==actionDatasetLength){
          actionRecordIdx =0;
          holdCount = 0;
          _controller->reset();
        }
      }

      DBG_INFO("Send Action to Joint.");
      if (_controller->getCurrIdx() > 1)
        actionSafetyCheck(robotAction);

//      std::cout<<"Time: "<<systemTime<<std::endl;
//      std::cout<<"jntPos: "<<robotState.motorPosition.transpose()<<std::endl;
//      std::cout<<"Action: "<<robotAction.motorPosition.transpose()<<std::endl;
//      std::cout<<"\n";

      _stateEstimator->print_estimate();
      setPositionReference(robotAction);
      DBG_INFO("Control Loop Finished.");
      systemTime += 1.0f / controlFreq;

    }

    CustomTypes::State getStateEstimation() {
      auto state = CustomTypes::zeroState();
      getMotorStates(state.motorPosition, state.motorVelocity);
      state.baseRotMat = ori::quaternionToRotationMatrixRsm(_stateEstimate.orientation);
      state.baseVelocity_w = _stateEstimate.vWorld;
      state.baseVelocity = _stateEstimate.vBody;
      state.baseRpyRate_w = _stateEstimate.omegaWorld;
      state.baseRpyRate = _stateEstimate.omegaBody;
      state.basePosition = _stateEstimate.position;
//      state.basePosition[2] += 0.03;
      state.phaseValue = _stateEstimate.phaseValue;
      return state;
    }

    void getMotorStates(Eigen::VectorXf &q, Eigen::VectorXf &qd) {
      for (uint i = 0; i < 12; i++) {
        q[i] = data.motorState[i].q;
        qd[i] = data.motorState[i].dq;
      }
    }

    void setPositionReference(const Eigen::VectorXd &position, const Eigen::VectorXd &kp, const Eigen::VectorXd &kd) {
      for (int i = 0; i < 12; i++) {
        cmd.motorCmd[i].q = position[i];
        cmd.motorCmd[i].mode = 10;
        cmd.motorCmd[i].Kp = kp[i];
        cmd.motorCmd[i].Kd = kd[i];
        cmd.motorCmd[i].tau = 0.;
      }
    }

    void setPositionReference(const CustomTypes::Action &ac) {
      for (uint i = 0; i < 12; i++) {
        cmd.motorCmd[i].q = ac.motorPosition[i];
        cmd.motorCmd[i].dq = ac.motorVelocity[i];
        cmd.motorCmd[i].mode = 10;
        cmd.motorCmd[i].Kp = ac.kP[i];
        cmd.motorCmd[i].Kd = ac.kD[i];
        cmd.motorCmd[i].tau = 0.;
      }
    }

    static void printState(CustomTypes::State st) {
      std::cout << "\npos: " << st.basePosition.transpose() <<std::endl;
      std::cout << "vWd: " << st.baseVelocity_w.transpose() <<std::endl;
      std::cout << "vBd: " << st.baseVelocity.transpose() <<std::endl;
      std::cout << "wWd: " << st.baseRpyRate_w.transpose() <<std::endl;
      std::cout << "wBd: " << st.baseRpyRate.transpose() <<std::endl;
      Mat3<float> rm = st.baseRotMat.block(0,0,3,3);
      std::cout << "Ori: " << ori::rotationMatrixToRPY(rm).transpose() <<std::endl;
    }

    /// Safety check for action. Called before action sent into interface.
    /// This function works only when getJointState() is called before in the timestep.
    /// Criterion:
    /// If joint position already out of limit, hold it back to limit with stiff position controller;
    /// If joint is approaching a target that is out of limit, shift the target to the border.
    void actionSafetyCheck(CustomTypes::Action &ac) {
      float cautionMargin = 0.1;
      float kPStiff = 300;
      float kDStiff = 15;
      for (uint i = 0; i < 12; i++) {
        if(_q[i]>_upperPosLimit[i]){
          ac.motorPosition[i] = _upperPosLimit[i];
          ac.motorVelocity[i] = 0.f;
          ac.kP[i] = kPStiff;
          ac.kD[i] = kDStiff;
          DBG_INFO("joint No."<<i<<" out of upper limit: "<<_q[i])
        }
        else if(_q[i]<_lowerPosLimit[i]){
          ac.motorPosition[i] = _lowerPosLimit[i];
          ac.motorVelocity[i] = 0.f;
          ac.kP[i] = kPStiff;
          ac.kD[i] = kDStiff;
          DBG_INFO("joint No."<<i<<" out of lower limit: "<<_q[i])
        }
        else if(ac.motorPosition[i]>_upperPosLimit[i] && _qd[i]>0 && _upperPosLimit[i]-_q[i]<cautionMargin) {
          ac.motorPosition[i] = _upperPosLimit[i];
          ac.motorVelocity[i] = 0.f;
          DBG_INFO("joint No."<<i<<" approaching upper limit, target "<<ac.motorPosition[i]<< "modified.")
        }
        else if(ac.motorPosition[i]<_lowerPosLimit[i] && _qd[i]<0 && _q[i]-_lowerPosLimit[i]<cautionMargin) {
          ac.motorPosition[i] = _lowerPosLimit[i];
          ac.motorVelocity[i] = 0.f;
          DBG_INFO("joint No."<<i<<" approaching lower limit, target "<<ac.motorPosition[i]<< "modified.")
        }
      }
    }

    bool phaseInitialized = false, _replayFlag=false;
    int nnCtrlIdx = 2, actionRecordIdx=0, actionDatasetLength=0;
    int hold_frames = 5, holdCount=0;
    float systemTime = 0., controlFreq=1e3;
    Eigen::VectorXf _q, _qd, _upperPosLimit, _lowerPosLimit;
    Eigen::MatrixXf _actionDataset;
    Vec3<float> linearVelTarg, angularVelTarg;
    CustomTypes::State robotState;
    Controller *_controller;
    StateEstimate<float> _stateEstimate;
    StateEstimatorContainer<float> *_stateEstimator;
    YAML::Node _yaml;

};

std::shared_ptr<Custom> io;

void signal_callback_handler(int signum) {
  printf("Interruped with SIGINT: %d", signum);
  io->Stop();
}

int main() {
  /* Register signal and signal handler */
  signal(SIGINT, signal_callback_handler); // put behind, otherwise will be overwirtten by others such as ROS

  YAML::Node yaml_root = YAML::LoadFile("./../config/params.yaml");
  io = std::make_shared<Custom>(yaml_root);
  io->Init();

  std::thread comm_thread(&Custom::Communicate, io);
  std::thread control_thread(&Custom::Control, io);

  comm_thread.join();
  control_thread.join();

  return 0;
}