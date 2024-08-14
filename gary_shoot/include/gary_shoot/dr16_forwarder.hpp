#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "gary_msgs/msg/dr16_receiver.hpp"
#include "std_msgs/msg/float64.hpp"
#include <string>
namespace gary_shoot{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    class DR16Forwarder : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit DR16Forwarder(const rclcpp::NodeOptions & options);


    private:

        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        std_msgs::msg::Float64 ShooterWheelOnMsg;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr ShooterWheelOnPublisher;
        std_msgs::msg::Float64 TriggerWheelOnMsg;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr TriggerWheelOnPublisher;


        void rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg);
        rclcpp::Subscription<gary_msgs::msg::DR16Receiver>::SharedPtr RemoteControlSubscription;

        void data_publisher();

        //timer
        rclcpp::TimerBase::SharedPtr timer_update;

        double update_freq;

        double shooter_wheel_pid_target;
        double trigger_wheel_pid_target;
        std::string remote_control_topic;
        std::string shooter_wheel_topic;
        std::string trigger_wheel_topic;
        std::uint8_t prev_switch_state;
        bool shooter_on;
        bool trigger_on;
    };
}
