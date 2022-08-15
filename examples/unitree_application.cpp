#include "types.h"
#include "hardware_interface.h"
#include "controller_interface.h"

#include "unitree_hardware.h"
#include "sine_controller.h"
#include <iostream>
#include "utils.h"

int main(int argc, char* argv[]) {

    interfaces::ControllerInterface* controller;
    interfaces::HardwareInterface* hardware;
    int num_iters;

    char * arg_controller_name = getCmdOption(argv, argv+argc, "--controller");
    if (arg_controller_name == nullptr) 
    {
        controller = new SineController();
    }
    
    char * arg_hardware_name = getCmdOption(argv, argv+argc, "--hardware");
    if (arg_hardware_name == nullptr)
    {
        hardware = new UnitreeHardware();
    }

    char * arg_num_iters = getCmdOption(argv, argv+argc, "--num-iters");
    if (arg_num_iters == nullptr)
    {
        num_iters = 3;
    }
    else
    {
        num_iters = std::stoi(arg_num_iters);
    }

    hardware->start();
    controller->start();

    // Initialize static memory
    static interfaces::RobotState robotState; 
    static interfaces::RobotCommand robotCommand;

    // Demo control loop
    for (int i =0; i < num_iters; i ++) 
    {
        hardware->read();
        hardware->getRobotState(robotState);
        controller->setRobotState(robotState);
        controller->step_control();
        controller->getRobotCommand(robotCommand);
        hardware->setRobotCommand(robotCommand);
        hardware->write();
        std::cout << "FR hip, knee qDes: " << robotCommand.legs[0].joints[1].q << " " << robotCommand.legs[0].joints[2].q << std::endl;
        std::cout << "Unitree application loop: " << i << std::endl;
    }

    hardware->stop();
    controller->stop();
    std::cout << "Unitree application terminated." << std::endl;

    return 0;
}