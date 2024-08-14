//
// Created by maackia on 23-2-7.
//

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "gary_msgs/msg/dr16_receiver.hpp"
#include "gary_msgs/msg/auto_aim.hpp"
#include "gary_gimbal/gimbal_enter.hpp"

using namespace std::chrono_literals;

class GimbalEnterTask : public rclcpp::Node
{
public:
    GimbalEnterTask():Node("gimbal_enter"){
        rc_sub_ = this->create_subscription<gary_msgs::msg::DR16Receiver>("/remote_control",rclcpp::SystemDefaultsQoS(),std::bind(&GimbalEnterTask::rc_callback,this,std::placeholders::_1));
        auto_aim_sub_ = this->create_subscription<gary_msgs::msg::AutoAIM>("/auto_aim",rclcpp::SystemDefaultsQoS(),std::bind(&GimbalEnterTask::auto_aim_callback,this,std::placeholders::_1));//TODO topic待定
        enter_publisher_ = this->create_publisher<std_msgs::msg::Int16>("/gimbal_enter",rclcpp::SystemDefaultsQoS());//TODO 信息类型待定
    }
private:
    void rc_callback(const gary_msgs::msg::DR16Receiver::SharedPtr msg){
        enter::RC_control = *msg;
        if (enter::RC_control.sw_right == enter::RC_control.SW_DOWN){
            enter::behaviour == GIMBAL_ZERO_FORCE;
        }
        else if (enter::RC_control.sw_right == enter::RC_control.SW_MID){
            enter::behaviour == GIMBAL_ABSOLUTE_ANGLE;
        }
        else if (enter::RC_control.sw_right == enter::RC_control.SW_UP){
            enter::behaviour == GIMBAL_RELATIVE_ANGLE;
        }

        if (enter::behaviour == GIMBAL_ABSOLUTE_ANGLE){
            std_msgs::msg::Int16 enter;
            enter.data = enter::behaviour;
            enter_publisher_->publish(enter);
            enter.data = enter::RC_control.ch_right_x;
            enter_publisher_->publish(enter);
            enter.data = enter::RC_control.ch_right_y;
            enter_publisher_->publish(enter);
        }
    }
    void auto_aim_callback(const gary_msgs::msg::AutoAIM::SharedPtr msg){
        enter::autoAim = *msg;
        if (enter::autoAim.target_id != enter::autoAim.TARGET_ID0_NONE && enter::RC_control.sw_right == enter::RC_control.SW_UP){
            std_msgs::msg::Int16 enter;
//            enter.data = enter::behaviour;
//            enter_publisher_->publish(enter);
            enter.data = enter::autoAim.yaw;
            enter_publisher_->publish(enter);
            enter.data = enter::autoAim.pitch;
            enter_publisher_->publish(enter);
        }
    }
    rclcpp::Subscription<gary_msgs::msg::DR16Receiver>::SharedPtr rc_sub_;
    rclcpp::Subscription<gary_msgs::msg::AutoAIM>::SharedPtr auto_aim_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr enter_publisher_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalEnterTask>());
    return 0;
}