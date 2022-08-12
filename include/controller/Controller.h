#ifndef CONTROLLER_INTERFACE_H
#define CONTROLLER_INTERFACE_H

#include <utility>

#include "types/CustomTypes.h"
#include "types/cpp_types.h"


#ifdef DEBUG
#define DBG_INFO(x) std::cout<<x<<std::endl;
#else
#define DBG_INFO(x)
#endif

/*  Abstract interface for all controllers */
class Controller
{
public: 
    const static size_t kNumMotors = 12;

    Controller(): _robotState(CustomTypes::zeroState()),_curr_idx(0),simulationTime(0.) {};
    virtual ~Controller() = default;;
    virtual void reset() = 0;
    virtual bool isComplete() const = 0; 
    virtual CustomTypes::Action getControlAction(const CustomTypes::State& robotState) = 0;
    size_t getCurrIdx() const {return _curr_idx;}

    virtual void setBaseVelTarget(Vec3<float> lin, Vec3<float> ang) {
      baseLinVelTarget = std::move(lin);
      baseAngVelTarget = std::move(ang);
    }

    virtual void sendTime(double t){simulationTime = t;}

private:
    CustomTypes::State _robotState;
protected:
    size_t _curr_idx;
    double simulationTime{};
    Vec3<float> baseLinVelTarget, baseAngVelTarget;

};



#endif