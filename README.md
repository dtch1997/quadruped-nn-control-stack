# Neural Locomotion for Unitree A1 

The nn_locomotion_unitree is a project to achieve locomotion with trained neural policies for Unitree A1 robot.
It is based on [unitree_legged_sdk V3.3.1](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.3.1) is mainly used for communication between PC and Controller board.
It also can be used in other PCs with UDP.

### Notice
support robot: A1(sport_mode >= v1.19)

### Dependencies
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [LCM](https://lcm-proj.github.io) (version 1.4.0 or higher)
```bash
cd lcm-x.x.x
mkdir build && cd build
cmake ..
make
sudo make install
```

### File Config

1. delete all build files from the project folder,
2. Copy the project folder to the robot via FTP,
3. Copy the policy weight folder to robot via FTP,
4. Reserve a directory for logging on the robot,
5. Open `./config/params.yaml`, rewrite the path to config directory, weights and logs to be consistent with step 2 to 4.


### Build
```bash
mkdir build && cd build
cmake ..
make
```

### Usage
Run examples with 'sudo' for memory locking.
