#include "types.h"
#include "hardware_interface.h"
#include "controller_interface.h"
#include <iostream>

class FakeHardware : public interfaces::HardwareInterface
{
public:
    int start() override {return 0;};
    int read() override {return 0;};
    int write() override {return 0;};
    int stop() override {return 0;};
};

class FakeController : public interfaces::ControllerInterface
{
public:
    int start() override {return 0;};
    int step_control() override {return 0;};
    int stop() override {return 0;};
};

int main() {
    // Demo fake hardware
    auto hardware = FakeHardware();
    hardware.start();
    std::cout << "Fake hardware initialized" << std::endl;

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
        std::cout << "Fake application loop: " << i << std::endl;
    }

    hardware.stop();
    controller.stop();
    std::cout << "Fake application terminated." << std::endl;

    return 0;
}