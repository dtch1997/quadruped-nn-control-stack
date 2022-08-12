#ifndef JOINT_COMMAND_HPP
#define JOINT_COMMAND_HPP

#include <eigen3/Eigen/Dense>
#include "rbdl/rbdl.h"

/// \namespace legged_robot
namespace legged_robot
{
struct JointCommand {
    RigidBodyDynamics::Math::VectorNd torque;
    RigidBodyDynamics::Math::VectorNd position;
};

}

#endif
