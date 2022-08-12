#ifndef COMMON__HARDWARE_INTERFACE
#define COMMON__HARDWARE_INTERFACE

#include "types.h"

namespace common
{
    class HardwareInterface
    {
    public:
        virtual int start() = 0;
        virtual int read() = 0;
        virtual int write() = 0;
        virtual int stop() = 0;

    protected:
        ImuState _imu_data;
        LegState _leg_states[4];
        LegCommand _leg_commands[4];
    };
} // namespace common

#endif // COMMON__HARDWARE_INTERFACE