#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <string>
#include <cmath>
#include <chrono>
#include <limits>

static inline bool DoubleEqual(double a, double b)
{
    return std::abs(a - b) < std::numeric_limits<double>::epsilon();
}

namespace gary_shoot{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class ShooterController : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit ShooterController(const rclcpp::NodeOptions & options);


    private:

        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        std_msgs::msg::Float64 LeftShooterWheelPIDMsg;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr LeftShooterWheelPIDPublisher;
        std_msgs::msg::Float64 RightShooterWheelPIDMsg;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr RightShooterWheelPIDPublisher;
        std_msgs::msg::Float64 TriggerWheelPIDMsg;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr TriggerWheelPIDPublisher;

        void shooter_callback(std_msgs::msg::Float64::SharedPtr msg);
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ShooterSubscription;
        void trigger_callback(std_msgs::msg::Float64::SharedPtr msg);
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr TriggerSubscription;
        void diag_callback(diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr DiagnosticSubscription;

        void data_publisher();

        //timer
        rclcpp::TimerBase::SharedPtr timer_update;

        double update_freq;

        std::string shooter_wheel_receive_topic;
        std::string trigger_wheel_receive_topic;
        std::string diagnostic_topic;

        std::string left_wheel_send_topic;
        std::string right_wheel_send_topic;
        double shooter_wheel_pid_target;
        double shooter_wheel_pid_current_set;
        std::string trigger_wheel_send_topic;
        double trigger_wheel_pid_target;
        double trigger_wheel_current_set;

        bool shooter_on;
        bool trigger_on;
        bool motor_offline;

        std::map<std::string,bool> diag_objs;
    };

}