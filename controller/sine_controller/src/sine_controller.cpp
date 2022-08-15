#include "sine_controller.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

int SineController::start()
{
    return 0;
}

int SineController::stop()
{
    return 0;
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

int SineController::step_control()
{
    motiontime++;

    // gravity compensation
    _robot_command.legs[0].joints[0].tau = -0.65f;
    _robot_command.legs[1].joints[0].tau = +0.65f;
    _robot_command.legs[2].joints[0].tau = -0.65f;
    _robot_command.legs[3].joints[0].tau = +0.65f;

    if( motiontime >= 0){
        // first, get record initial position
        if( motiontime >= 0 && motiontime < 10){
            qInit[0] = _robot_state.legs[0].joints[0].q;
            qInit[1] = _robot_state.legs[0].joints[1].q;
            qInit[2] = _robot_state.legs[0].joints[2].q;
        }
        // second, move to the origin point of a sine movement with Kp Kd
        if( motiontime >= 10 && motiontime < 400){
            rate_count++;
            double rate = rate_count/200.0;                       // needs count to 200
            Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0; 
            Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
            
            qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
            qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
            qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
        }
        double sin_joint1, sin_joint2;
        // last, do sine wave
        if( motiontime >= 400){
            sin_count++;
            sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
            sin_joint2 = -0.6 * sin(1.8*M_PI*sin_count/1000.0);
            qDes[0] = sin_mid_q[0];
            qDes[1] = sin_mid_q[1];
            qDes[2] = sin_mid_q[2] + sin_joint2;
            // qDes[2] = sin_mid_q[2];
        }

        _robot_command.legs[0].joints[0].q = qDes[0];
        _robot_command.legs[0].joints[0].qd = 0;
        _robot_command.legs[0].joints[0].kp = Kp[0];
        _robot_command.legs[0].joints[0].kd = Kd[0];
        _robot_command.legs[0].joints[0].tau = -0.65f;

        _robot_command.legs[0].joints[1].q = qDes[1];
        _robot_command.legs[0].joints[1].qd = 0;
        _robot_command.legs[0].joints[1].kp = Kp[1];
        _robot_command.legs[0].joints[1].kd = Kd[1];
        _robot_command.legs[0].joints[1].tau = 0.0f;

        _robot_command.legs[0].joints[2].q =  qDes[2];
        _robot_command.legs[0].joints[2].qd = 0;
        _robot_command.legs[0].joints[2].kp = Kp[2];
        _robot_command.legs[0].joints[2].kd = Kd[2];
        _robot_command.legs[0].joints[2].tau = 0.0f;
    }

    return 0;
}
