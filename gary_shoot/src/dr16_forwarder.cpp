#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "gary_msgs/msg/dr16_receiver.hpp"
#include "std_msgs/msg/float64.hpp"
#include "gary_shoot/dr16_forwarder.hpp"
#include <string>
#include <chrono>

namespace gary_shoot{

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    DR16Forwarder::DR16Forwarder(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode(
            "dr16_forwarder", options) {
        this->declare_parameter("update_freq", 100.0f);
        this->declare_parameter("remote_control_topic", "/remote_control");
        this->declare_parameter("shooter_wheel_topic", "/rc_shooter_pid_set");
        this->declare_parameter("trigger_wheel_topic", "/rc_trigger_pid_set");
        this->declare_parameter("shooter_wheel_pid_target", 8500.0f);
        this->declare_parameter("trigger_wheel_pid_target", 3000.0f);

        this->remote_control_topic = "/remote_control";

        this->shooter_wheel_topic = "/rc_shooter_pid_set";
        this->trigger_wheel_topic= "/rc_trigger_pid_set";

        this->shooter_wheel_pid_target = static_cast<double>(8500.0f);
        this->trigger_wheel_pid_target = static_cast<double>(3000.0f);

        this->update_freq = 100.0f;
        this->prev_switch_state = 0;
        this->shooter_on = false;
        this->trigger_on = false;
    }

    CallbackReturn DR16Forwarder::on_configure(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if (this->get_parameter("update_freq").get_type() != rclcpp::PARAMETER_DOUBLE) {
            RCLCPP_ERROR(this->get_logger(), "update_freq type must be double");
            return CallbackReturn::FAILURE;
        }
        this->update_freq = this->get_parameter("update_freq").as_double();


        if(this->get_parameter("shooter_wheel_pid_target").get_type() != rclcpp::PARAMETER_DOUBLE){
            RCLCPP_ERROR(this->get_logger(), "TYPE ERROR: \"shooter_wheel_pid_target\" must be double.");
            return CallbackReturn::FAILURE;
        }
        shooter_wheel_pid_target = abs(this->get_parameter("shooter_wheel_pid_target").as_double());

        if(this->get_parameter("trigger_wheel_pid_target").get_type() != rclcpp::PARAMETER_DOUBLE){
            RCLCPP_ERROR(this->get_logger(), "TYPE ERROR: \"trigger_wheel_pid_target\" must be double.");
            return CallbackReturn::FAILURE;
        }
        trigger_wheel_pid_target = abs(this->get_parameter("trigger_wheel_pid_target").as_double());


        if(this->get_parameter("shooter_wheel_topic").get_type() != rclcpp::PARAMETER_STRING){
            RCLCPP_ERROR(this->get_logger(), "TYPE ERROR: \"shooter_wheel_topic\" must be a string.");
            return CallbackReturn::FAILURE;
        }
        shooter_wheel_topic = this->get_parameter("shooter_wheel_topic").as_string();
        ShooterWheelOnPublisher =
                create_publisher<std_msgs::msg::Float64>(shooter_wheel_topic,rclcpp::SystemDefaultsQoS());

        if(this->get_parameter("trigger_wheel_topic").get_type() != rclcpp::PARAMETER_STRING){
            RCLCPP_ERROR(this->get_logger(), "TYPE ERROR: \"trigger_wheel_topic\" must be a string.");
            return CallbackReturn::FAILURE;
        }
        trigger_wheel_topic = this->get_parameter("trigger_wheel_topic").as_string();
        TriggerWheelOnPublisher =
                create_publisher<std_msgs::msg::Float64>(trigger_wheel_topic,rclcpp::SystemDefaultsQoS());


        if(this->get_parameter("remote_control_topic").get_type() != rclcpp::PARAMETER_STRING){
            RCLCPP_ERROR(this->get_logger(), "TYPE ERROR: \"remote_control_topic\" must be a string.");
            return CallbackReturn::FAILURE;
        }
        remote_control_topic = this->get_parameter("remote_control_topic").as_string();
        this->RemoteControlSubscription =
                this->create_subscription<gary_msgs::msg::DR16Receiver>(
                        remote_control_topic,
                        rclcpp::SystemDefaultsQoS(),
                        std::bind(&DR16Forwarder::rc_callback,this,std::placeholders::_1));


        RCLCPP_INFO(this->get_logger(), "configured");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DR16Forwarder::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        timer_update->reset();
        TriggerWheelOnPublisher.reset();
        ShooterWheelOnPublisher.reset();
        RemoteControlSubscription.reset();

        RCLCPP_INFO(this->get_logger(), "cleaned up");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DR16Forwarder::on_activate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);
        using namespace std::chrono_literals;
        timer_update = this->create_wall_timer(1000ms/this->update_freq,[this] { data_publisher(); });
        TriggerWheelOnPublisher->on_activate();
        ShooterWheelOnPublisher->on_activate();
        RCLCPP_INFO(this->get_logger(), "activated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DR16Forwarder::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        timer_update->reset();
        TriggerWheelOnPublisher->on_deactivate();
        ShooterWheelOnPublisher->on_deactivate();
        RemoteControlSubscription.reset();

        RCLCPP_INFO(this->get_logger(), "deactivated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DR16Forwarder::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if(timer_update.get()!=nullptr) timer_update->reset();
        if(TriggerWheelOnPublisher.get() != nullptr) TriggerWheelOnPublisher.reset();
        if(ShooterWheelOnPublisher.get() != nullptr) ShooterWheelOnPublisher.reset();
        if(RemoteControlSubscription.get() != nullptr) RemoteControlSubscription.reset();

        RCLCPP_INFO(this->get_logger(), "shutdown");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DR16Forwarder::on_error(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if(timer_update.get()!=nullptr) timer_update->reset();
        if(TriggerWheelOnPublisher.get() != nullptr) TriggerWheelOnPublisher.reset();
        if(ShooterWheelOnPublisher.get() != nullptr) ShooterWheelOnPublisher.reset();
        if(RemoteControlSubscription.get() != nullptr) RemoteControlSubscription.reset();

        RCLCPP_ERROR(this->get_logger(), "Error happened");
        return CallbackReturn::SUCCESS;
    }

    void DR16Forwarder::rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg) {
        // I know "msg" should be made const, but it somehow goes error, so do not change it.
        std::uint8_t switch_state = 0;
        switch_state = msg->sw_left;
        if(prev_switch_state == msg->SW_MID) {
            if(switch_state == msg->SW_DOWN){
                trigger_on = !trigger_on;
                RCLCPP_INFO(this->get_logger(),trigger_on?"Trigger on!":"Trigger off!");
            }else if (switch_state == msg->SW_UP) {
                shooter_on = !shooter_on;
                RCLCPP_INFO(this->get_logger(),shooter_on?"Shooter on!":"Shooter off!");
            }
            if(trigger_on && !shooter_on){
                trigger_on = false;
                RCLCPP_WARN(this->get_logger(),"Trigger off!: cannot turn trigger on while shooter is off!");
            }
        }
        if(msg->sw_right == msg->SW_DOWN){
            if(shooter_on) {
                shooter_on = false;
                RCLCPP_WARN(this->get_logger(),"Shooter off! Zero force!");
            }
            if(trigger_on) {
                trigger_on = false;
                RCLCPP_WARN(this->get_logger(),"Trigger off! Zero force!");
            }
        }
        prev_switch_state = switch_state;
    }

    void DR16Forwarder::data_publisher() {
        if(this->trigger_on && this->shooter_on){
            TriggerWheelOnMsg.data = this->trigger_wheel_pid_target;
        }else{
            TriggerWheelOnMsg.data = static_cast<double>(0.0f);
        }
        if(this->shooter_on){
            ShooterWheelOnMsg.data = this->shooter_wheel_pid_target;
        }else{
            ShooterWheelOnMsg.data = static_cast<double>(0.0f);
        }

        ShooterWheelOnPublisher->publish(ShooterWheelOnMsg);
        TriggerWheelOnPublisher->publish(TriggerWheelOnMsg);
    }
}


int main(int argc, char const *const argv[]){
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<gary_shoot::DR16Forwarder> dr16Forwarder =
            std::make_shared<gary_shoot::DR16Forwarder>(rclcpp::NodeOptions());

    exe.add_node(dr16Forwarder->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_shoot::DR16Forwarder)