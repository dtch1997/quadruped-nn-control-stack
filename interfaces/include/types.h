// Define various types used in abstract interface

#ifndef COMMON__TYPES
#define COMMON__TYPES

namespace common
{
    struct ImuState
    {
        double quad[4] = {0};
        double gyro[3] = {0};
        double acc[3] = {0};
    };

    struct JointState
    {
        double q = 0;   // motor current position (rad)
        double qd = 0;  // motor current speed（rad/s）
        double tau = 0; // current estimated output torque（N*m）
    };
    
    struct LegState
    {
        JointState abad;
        JointState hip;
        JointState knee;
    };

    struct JointCommand
    {
        double q = 0;   // motor target position
        double qd = 0;  // motor target velocity
        double tau = 0; // motor target torque
        double kp = 0;  // motor spring stiffness coefficient
        double kd = 0;  // motor damper coefficient
    };

    struct LegCommand
    {
        JointCommand abad;
        JointCommand hip;
        JointCommand knee;
    };
} // namespace dogotix_control

#endif // COMMON__TYPES