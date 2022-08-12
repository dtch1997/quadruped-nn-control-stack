#include "types.h"
#include "hardware_interface.h"
#include "controller_interface.h"

#include "unitree_hardware.h"
#include <iostream>

class FakeController : public interfaces::ControllerInterface
{
public:
    int start() override {return 0;};
    int step_control() override {return 0;};
    int stop() override {return 0;};
};

int main() {
    // Demo fake hardware
    auto hardware = UnitreeHardware();
    hardware.start();
    std::cout << "Unitree hardware initialized" << std::endl;

    // Demo fake controller
    auto controller = FakeController();
    controller.start();
    std::cout << "Fake controller initialized" << std::endl;

    // Initialize static memory
    static interfaces::RobotState robotState; 
    static interfaces::RobotCommand robotCommand;

    // Demo control loop
    for (int i =0; i < 3; i ++) 
    {
        hardware.read();
        hardware.getRobotState(robotState);
        controller.setRobotState(robotState);
        controller.step_control();
        controller.getRobotCommand(robotCommand);
        hardware.setRobotCommand(robotCommand);
        hardware.write();
        std::cout << "Unitree application loop: " << i << std::endl;
    }

    hardware.stop();
    controller.stop();
    std::cout << "Unitree application terminated." << std::endl;

    return 0;
}