//
// Created by maackia on 23-2-7.
//

#ifndef BUILD_GIMBAL_ENTER_HPP
#define BUILD_GIMBAL_ENTER_HPP

typedef enum
{
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_ABSOLUTE_ANGLE,
    GIMBAL_RELATIVE_ANGLE,
    GIMBAL_INIT,
    GIMBAL_CALI,
    GIMBAL_MOTIONLESS,
} gimbal_behaviour_e;

namespace enter
{
    gary_msgs::msg::DR16Receiver RC_control;
    gary_msgs::msg::AutoAIM autoAim;
    gimbal_behaviour_e behaviour;
    gimbal_behaviour_e last_behaviour;
}


#endif //BUILD_GIMBAL_ENTER_HPP
