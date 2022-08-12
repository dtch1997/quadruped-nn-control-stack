// Define various types used in abstract interface

#ifndef INTERFACES__TYPES
#define INTERFACES__TYPES

namespace interfaces
{
    struct ImuState
    {
        // w, x, y, z
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
        // 0 = ab adduction/abduction
        // 1 = hip flexion/extension
        // 2 = knee flexion/extension
        JointState joints[3];
    };

    struct RobotState
    {
        // 0 = FR
        // 1 = FL
        // 2 = RR
        // 3 = RL
        LegState legs[4];
        ImuState imu;
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
        // 0 = abdominal adduction/abduction
        // 1 = hip flexion/extension
        // 2 = knee flexion/extension
        JointCommand joints[3];
    };

    struct RobotCommand
    {
        // 0 = FR
        // 1 = FL
        // 2 = RR
        // 3 = RL
        LegCommand legs[4];
    };

} // namespace interfaces

#endif // INTERFACES__TYPES