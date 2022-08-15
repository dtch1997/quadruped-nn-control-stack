#pragma once

#include <eigen3/Eigen/Dense>
#include <map>
#include <iostream>

/// \namespace legged_robot
namespace legged_robot
{
struct Pose {
    Eigen::Vector3d position;
    Eigen::Matrix3d orientation;

    Pose() {
        position = Eigen::Vector3d::Zero();
        orientation = Eigen::Matrix3d::Identity();
    }
};

struct Twist {
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;

    Twist() {
        linear = Eigen::Vector3d::Zero();
        angular = Eigen::Vector3d::Zero();
    }
};

struct Accel {
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;

    Accel() {
        linear = Eigen::Vector3d::Zero();
        angular = Eigen::Vector3d::Zero();
    }
};

struct Wrench {
    Eigen::Vector3d force;
    Eigen::Vector3d torque;

    Wrench() {
        force = Eigen::Vector3d::Zero();
        torque = Eigen::Vector3d::Zero();
    }
};

struct Transform {
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;

    Transform() {
        translation = Eigen::Vector3d::Zero();
        rotation = Eigen::Matrix3d::Identity();
    }
};

struct MotionFrame {
    Pose pose;
    Twist twist;
    Accel accel;
    bool contact_status;
    double spinning_vel;

    MotionFrame() {
        contact_status = true;
        spinning_vel = 0.0;
    }
};

typedef std::map<std::string, MotionFrame> MotionFrameMap;

class StampedMotionFrameMap
{
public:
    MotionFrameMap motion_frame_map;
    double current_phase_time;
    std::string current_swing_leg;

    double swing_time;
    double switch_time;

    Eigen::Vector3d desired_vel;

    double forward_vel;
    double lateral_vel;
    double turning_vel;

    Eigen::Vector2d center_xy;
    double center_yaw;
    bool stop_flag;

//     double (* terrain)(double& x, double& y);
    std::function<double ( double& x, double& y ) > terrain;

    StampedMotionFrameMap() {
        Stop();
    };
    ~StampedMotionFrameMap() {};

    virtual void Next() {
        std::cout << "Do NOT use virtual motion!" << std::endl;
    };

    void Start() {
        stop_flag = false;
        current_phase_time = swing_time;
    };

    void Stop() {
        forward_vel = 0.0;
        lateral_vel = 0.0;
        turning_vel = 0.0;
        stop_flag = true;
    };

    Eigen::Vector3d GetZmpRef() {
        Eigen::Vector3d zmp_ref = Eigen::Vector3d::Zero();
        unsigned int contact_num = 0;
        for ( auto &it:motion_frame_map ) {
            if ( it.first != "CoM" && it.second.contact_status ) {
                contact_num += 1;
                zmp_ref += it.second.pose.position;
            }
        }
//         zmp_ref[0] = zmp_ref[0]/contact_num;
//         zmp_ref[1] = zmp_ref[1]/contact_num;
//         zmp_ref[2] = 0;
        zmp_ref = 1.0/contact_num*zmp_ref;

        return zmp_ref;
    };

    Eigen::Vector3d GetZmp ( const double &g = 9.81 ) {
        return motion_frame_map["CoM"].pose.position - motion_frame_map["CoM"].accel.linear * ( motion_frame_map["CoM"].pose.position[2]-GetZmpRef() [2] ) / g;
    };

    void Print() {
        for ( auto &it:motion_frame_map ) {
            std::cout << it.first << ": " << it.second.pose.position.transpose() << std::endl;
        }
    }
};

}
