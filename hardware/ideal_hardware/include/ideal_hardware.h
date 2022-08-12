// An ideal hardware that instantly reaches the commanded pos, vel, tau
// Useful for debugging

#include "hardware_interface.h"

class IdealHardware : public interfaces::HardwareInterface
{
public:
    int start() override; 
    int read() override;
    int write() override; 
    int stop() override;
};