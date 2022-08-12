#include "ideal_hardware.h"
#include <stdint.h>

// Public API

int IdealHardware::start()
{
    return 0;
};

int IdealHardware::stop()
{
    return 0;
}

int IdealHardware::read() 
{
    return 0;
}

int IdealHardware::write() 
{
    // Update cmd_ from _robot_command
    for (unsigned int leg = 0; leg<4; leg++)
    {
        for(unsigned int jindx = 0; jindx<3; jindx++)
        {
            _robot_state.legs[leg].joints[jindx].q = _robot_command.legs[leg].joints[jindx].q;
            _robot_state.legs[leg].joints[jindx].qd = _robot_command.legs[leg].joints[jindx].qd;
            _robot_state.legs[leg].joints[jindx].tau = _robot_command.legs[leg].joints[jindx].tau;
        }
    }

    return 0;
}