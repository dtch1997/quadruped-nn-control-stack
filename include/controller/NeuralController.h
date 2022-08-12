#ifndef OBS_REPLAY_CONTROLLER_H
#define OBS_REPLAY_CONTROLLER_H

#include "controller/Controller.h"
#include "types/cpp_types.h"
#include "MiniDNN/Actor.h"
#include "utils/orientation_tools.h"
#include <cassert>
#include <cstdio>

/*  Controller that performs neural net inference */ 
class NeuralController : public Controller 
{
public: 
    NeuralController(const std::string& save_path, const std::string& net_path);
    void reset() override;
    bool isComplete() const override;

    void updateObservation(const CustomTypes::State& robotState);

    CustomTypes::Action getControlAction(const CustomTypes::State& robotState) override;

    CustomTypes::Action constructAction(const Eigen::VectorXd& jntPosTarg) const;

    Eigen::VectorXf jntPGain, jntDGain;
    Eigen::MatrixXf observation;
    Eigen::MatrixXd action;
    Mat3<float> bOri_rm;
    Vec3<float> curVelTarg;
    Vec3<float> bVel_w, bOmg_w;
    int obDim, acDim;
    float ctrlFreq;
    float accMax;

private:

    static void jointOrderReConstruct(Eigen::VectorXf &jntInfo);
    static void jointOrderReConstruct(Eigen::MatrixXd &jntInfo);

    // Simulated observation replay buffer
    int timestep = 0;
    std::vector<CustomTypes::State> robotStates;
    MLP_Actor actor;
    std::string netPath, logPath;
    Eigen::MatrixXd actionDataset;

};

#endif