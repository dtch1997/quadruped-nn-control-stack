#ifndef INTERFACES__HARDWARE_INTERFACE
#define INTERFACES__HARDWARE_INTERFACE

#include "types.h"

namespace interfaces
{
    class HardwareInterface
    {
    public:
        virtual int start() = 0;
        // Read state from hardware into _robotState
        virtual int read() = 0;
        // Write command from _robot_command into hardware
        virtual int write() = 0;
        virtual int stop() = 0;

        void getRobotState(RobotState& state) const 
        {
            state = _robot_state;
        };
        void setRobotCommand(const RobotCommand& cmd) 
        {
            _robot_command = cmd;
        };

    protected:
        RobotState _robot_state;
        RobotCommand _robot_command;
    };
} // namespace interfaces

#endif // INTERFACES__HARDWARE_INTERFACE