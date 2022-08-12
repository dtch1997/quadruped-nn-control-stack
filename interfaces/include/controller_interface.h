#ifndef INTERFACES__CONTROLLER_INTERFACE
#define INTERFACES__CONTROLLER_INTERFACE

#include "types.h"

namespace interfaces
{
    class ControllerInterface
    {
    public:
        virtual int start() = 0;
        // For _robot_state, compute desired _robot_command
        virtual int step_control() = 0;
        virtual int stop() = 0;

        void setRobotState(const RobotState& state)
        {
            _robot_state = state;
        };
        RobotCommand getRobotCommand(RobotCommand& cmd) const
        {
            cmd = _robot_command;
        };

    protected:
        RobotState _robot_state;
        RobotCommand _robot_command;
    };
} // namespace interfaces

#endif // INTERFACES__CONTROLLER_INTERFACE