#pragma once

namespace legged_robot
{
    /* definition of each leg and joint */
    constexpr int FL_ = 0;       // leg index
    constexpr int FR_ = 1;
    constexpr int RL_ = 2;
    constexpr int RR_ = 3;

    constexpr int FL_0 = 0;      // joint index
    constexpr int FL_1 = 1;      
    constexpr int FL_2 = 2;

    constexpr int FR_0 = 3;
    constexpr int FR_1 = 4;
    constexpr int FR_2 = 5;

    constexpr int RL_0 = 6;
    constexpr int RL_1 = 7;
    constexpr int RL_2 = 8;

    constexpr int RR_0 = 9;
    constexpr int RR_1 = 10;
    constexpr int RR_2 = 11;

}

enum class RobotType { UNITREE_A1, UNITREE_ALIENGO, CHEETAH_3, MINI_CHEETAH };

