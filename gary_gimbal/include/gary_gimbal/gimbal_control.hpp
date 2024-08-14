//
// Created by maackia on 23-2-7.
//

#ifndef BUILD_GIMBAL_CONTROL_HPP
#define BUILD_GIMBAL_CONTROL_HPP

#define PI 3.141592f

typedef enum
{
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_ABSOLUTE_ANGLE,
    GIMBAL_INIT,
    GIMBAL_CALI,
    GIMBAL_RELATIVE_ANGLE,
    GIMBAL_MOTIONLESS,
} gimbal_behaviour_e;

namespace gimbal
{
    sensor_msgs::msg::Imu Imu;//TODO 初始化
    std_msgs::msg::Int16 state;
    gary_msgs::msg::DualLoopPIDwithFilter yaw_pid;
    gary_msgs::msg::DualLoopPIDwithFilter pitch_pid;

    class Mode
    {
    public:
        gimbal_behaviour_e behaviour;
        gimbal_behaviour_e last_behaviour;
    };

    class Motor : public Mode{
    public:
        double absolute_angle;
        double absolute_angle_set;
        double absolute_angle_max;
        double absolute_angle_min;
        double relative_angle;

        double ecd_transform;
        double ecd_set;
        double ecd_delta;
        double pid_set;

    };

    Motor yaw;
    Motor pitch;
    Mode mode;

}

#endif //BUILD_GIMBAL_CONTROL_HPP
