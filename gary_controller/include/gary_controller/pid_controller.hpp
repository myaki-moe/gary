#pragma once

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64.hpp"
#include "gary_msgs/msg/pid.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace gary_msgs::msg;

namespace gary_controller {

class PIDController : public controller_interface::ControllerInterface {

public:

    PIDController();


    controller_interface::return_type init(const std::string &controller_name) override;


    controller_interface::InterfaceConfiguration command_interface_configuration() const override;


    controller_interface::InterfaceConfiguration state_interface_configuration() const override;


    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;


    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;


    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;


    controller_interface::return_type update() override;

private:

    bool check_parameter(const std::string& name, rclcpp::ParameterType type);

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    //params
    std::string command_interface_name;
    std::string state_interface_name;
    PID pid;
    double stale_threshold;

    //publisher and subscriber
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_subscription;
    std::unique_ptr<realtime_tools::RealtimePublisher<gary_msgs::msg::PID>> pid_publisher;
    //rt buffer
    realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> cmd_buffer;
    double last_cmd_time;
};
}
