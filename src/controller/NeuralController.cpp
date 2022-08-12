#include "controller/NeuralController.h"

//#define ACTION_REPLAY

/*  Controller based on replaying actions form the robot */
NeuralController::NeuralController(const std::string &save_path, const std::string &net_path):
  timestep(0){
  curVelTarg.setZero();
  ctrlFreq = 100;
  accMax = 0.6f / ctrlFreq;
  baseLinVelTarget.setZero();
  netPath = net_path;
  logPath = save_path + "/log_data.csv";

  std::ofstream logFile;
  logFile.open(logPath,std::ios::out);
  if(logFile.is_open()) logFile.close();

  DBG_INFO("LOAD MiniDNN...");
  actor.deployNetwork(netPath);
  obDim = actor.n_input;
  acDim = actor.n_output;
  observation.setZero(obDim, 1);
  action.setZero(acDim, 1);

  DBG_INFO("LOAD Gain Data...");
  jntPGain = MLP_Actor::readCSV(netPath + "jointPgain.csv", acDim, 1).cast<float>();
  jntDGain = MLP_Actor::readCSV(netPath + "jointDgain.csv", acDim, 1).cast<float>();
#ifdef ACTION_REPLAY
  actionDataset = MLP_Actor::readCSV(netPath + "acSet_pure.csv", 500, acDim);
#endif
  DBG_INFO("jntPGain: " << jntPGain.transpose());
  DBG_INFO("jntDGain: " << jntDGain.transpose());
  DBG_INFO("LOAD Finish..");
}

void NeuralController::reset() {
  timestep = 0;
  curVelTarg.setZero();
}

bool NeuralController::isComplete() const { return false; }

void NeuralController::updateObservation(const CustomTypes::State &robotState) {

  Eigen::VectorXf jntPos_URDFOrder,jntVel_URDFOrder;
  jntPos_URDFOrder.setZero(12);
  jntVel_URDFOrder.setZero(12);
  jntPos_URDFOrder = robotState.motorPosition;
  jntVel_URDFOrder = robotState.motorVelocity;
//  jointOrderReConstruct(jntPos_URDFOrder);
//  jointOrderReConstruct(jntVel_URDFOrder);

  observation.setZero(obDim, 1);
  observation(0, 0) = robotState.basePosition[2];
  observation.col(0).segment(1, 3) = robotState.baseRotMat.row(2).transpose();
  observation.col(0).segment(16, 3) = robotState.baseVelocity;
  observation.col(0).segment(19, 3) = robotState.baseRpyRate;
  observation.col(0).segment(4, 12) = jntPos_URDFOrder;
  observation.col(0).segment(25, 12) = jntVel_URDFOrder;
  if (abs(curVelTarg[0] - baseLinVelTarget[0]) > 1e-3) curVelTarg[0] += (baseLinVelTarget - curVelTarg).cwiseSign()[0] * accMax;
//  observation.col(0).segment(22, 3) = robotState.baseRotMat.transpose() * curVelTarg;
  observation.col(0).segment(22, 3) = curVelTarg;

  observation(37, 0) = robotState.phaseValue;
  bVel_w = robotState.baseVelocity_w;
  bOmg_w = robotState.baseRpyRate_w;
  bOri_rm = robotState.baseRotMat;
}

CustomTypes::Action NeuralController::constructAction(const Eigen::VectorXd &jntPosTarg) const {
  auto robotAction = CustomTypes::zeroAction();
  robotAction.motorPosition = jntPosTarg.cast<float>();
  robotAction.kP = jntPGain;
  robotAction.kD = jntDGain;
  return robotAction;
}

CustomTypes::Action NeuralController::getControlAction(const CustomTypes::State &robotState) {
  updateObservation(robotState);
  actor.getAction(observation.cast<double>(), action);

#ifdef ACTION_REPLAY
  if(timestep<actionDataset.rows())
    action = actionDataset.block(timestep,0,1,acDim).transpose();
#endif

  if (observation(3, 0) > 0.4 && observation(0, 0) > 0.15) {
    std::cout << "time = " << simulationTime<<","<<timestep <<std::endl;
    std::cout << "target = " << baseLinVelTarget.transpose() <<std::endl;
    std::cout << "ob=np.array([";
    for (int i(0); i < obDim - 1; i++) std::cout << observation(i, 0) << ", ";
    std::cout << observation(obDim - 1, 0) << "])\nac=np.array([";
    for (int i(0); i < acDim - 1; i++) std::cout << action(i, 0) << ", ";
    std::cout << action(acDim - 1, 0) << "])\n";
  }

  std::ofstream logFile;
  logFile.open(logPath,std::ios::app);
  if(logFile.is_open()){
    auto bOri_eu = ori::rotationMatrixToRPY(bOri_rm);
    auto bOri_quat = ori::rotationMatrixToQuaternion(bOri_rm);
    logFile << simulationTime << ",";
    for (int i(0); i < obDim; i++) logFile << observation(i, 0) << ",";
    for (int i(0); i < acDim; i++) logFile << action(i, 0) << ",";
    for (int i(0); i < 3; i++) logFile << bVel_w[i] << ",";
    for (int i(0); i < 3; i++) logFile << bOmg_w[i] << ",";
    for (int i(0); i < 3; i++) logFile << bOmg_w[i] << ",";
    for (int i(0); i < 3; i++) logFile << bOri_eu[i] << ",";
    for (int i(0); i < 4; i++) logFile << bOri_quat[i] << ",";
    for (int i(0); i < 3; i++)
      for (int j(0); j < 3; j++)
        logFile << bOri_rm(i,j) << ",";
    if (observation(3, 0) > 0.4 && observation(0, 0) > 0.15) logFile << "1\n";
    else logFile << "0\n";
    logFile.close();
  }
//  jointOrderReConstruct(action);

  auto acStruct = constructAction(action);

  timestep += 1;
  return acStruct;
}

void NeuralController::jointOrderReConstruct(Eigen::VectorXf &jntInfo){
  assert(jntInfo.size()==12);
  Vec3<float> temp;
  temp = jntInfo.segment(3,3);
  jntInfo.segment(3,3) = jntInfo.segment(0,3);
  jntInfo.segment(0,3) = temp;
  temp = jntInfo.segment(6,3);
  jntInfo.segment(6,3) = jntInfo.segment(9,3);
  jntInfo.segment(9,3) = temp;
}

void NeuralController::jointOrderReConstruct(Eigen::MatrixXd &jntInfo){
  assert(jntInfo.cols()==12);
  Eigen::Matrix<double,3,1> temp;
  temp = jntInfo.block(0,0,3,1);
  jntInfo.block(0,0,3,1) = jntInfo.block(3,0,3,1);
  jntInfo.block(3,0,3,1) = temp;
  temp = jntInfo.block(6,0,3,1);
  jntInfo.block(6,0,3,1) = jntInfo.block(9,0,3,1);
  jntInfo.block(9,0,3,1) = temp;
}
