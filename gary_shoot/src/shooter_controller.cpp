#include "gary_shoot/shooter_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <string>
#include <cmath>
#include <chrono>
#include <limits>

namespace gary_shoot{

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    ShooterController::ShooterController(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode(
            "shooter_controller", options) {
        this->declare_parameter("update_freq", 100.0f);
        this->declare_parameter("shooter_wheel_receive_topic", "/shooter_wheel_controller_pid_set");
        this->declare_parameter("trigger_wheel_receive_topic", "/trigger_wheel_controller_pid_set");
        this->declare_parameter("left_shooter_wheel_send_topic", "/fric_left_pid/cmd");
        this->declare_parameter("right_shooter_wheel_send_topic", "/fric_right_pid/cmd");
        this->declare_parameter("trigger_wheel_send_topic", "/trigger_pid/cmd");
        this->declare_parameter("left_shooter_wheel_diag_name", "fric_left");
        this->declare_parameter("right_shooter_wheel_diag_name", "fric_right");
        this->declare_parameter("trigger_wheel_diag_name", "trigger");
        this->declare_parameter("diagnostic_topic", "/diagnostics_agg");

        this->shooter_wheel_receive_topic = "/shooter_wheel_controller_pid_set";
        this->trigger_wheel_receive_topic = "/trigger_wheel_controller_pid_set";
        this->left_wheel_send_topic = "/fric_left_pid/cmd";
        this->right_wheel_send_topic = "/fric_right_pid/cmd";
        this->shooter_wheel_pid_target = static_cast<double>(0.0f);
        this->trigger_wheel_send_topic = "/trigger_pid/cmd";
        this->trigger_wheel_pid_target = static_cast<double>(0.0f);

        this->diagnostic_topic = "/diagnostics_agg";
        this->motor_offline = true;

        this->update_freq = 100.0f;
        this->shooter_on = false;
        this->trigger_on = false;
        this->shooter_wheel_pid_current_set = static_cast<double>(0.0f);
        this->trigger_wheel_current_set = static_cast<double>(0.0f);

        diag_objs.clear();

    }

    CallbackReturn ShooterController::on_configure(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if (this->get_parameter("update_freq").get_type() != rclcpp::PARAMETER_DOUBLE) {
            RCLCPP_ERROR(this->get_logger(), "update_freq type must be double");
            return CallbackReturn::FAILURE;
        }
        this->update_freq = this->get_parameter("update_freq").as_double();

        if(this->get_parameter("left_shooter_wheel_send_topic").get_type() != rclcpp::PARAMETER_STRING){
            RCLCPP_ERROR(this->get_logger(), "TYPE ERROR: \"left_shooter_wheel_send_topic\" must be a string.");
            return CallbackReturn::FAILURE;
        }
        left_wheel_send_topic = this->get_parameter("left_shooter_wheel_send_topic").as_string();
        LeftShooterWheelPIDPublisher =
                create_publisher<std_msgs::msg::Float64>(left_wheel_send_topic,rclcpp::SystemDefaultsQoS());

        if(this->get_parameter("right_shooter_wheel_send_topic").get_type() != rclcpp::PARAMETER_STRING){
            RCLCPP_ERROR(this->get_logger(), "TYPE ERROR: \"right_shooter_wheel_send_topic\" must be a string.");
            return CallbackReturn::FAILURE;
        }
        right_wheel_send_topic = this->get_parameter("right_shooter_wheel_send_topic").as_string();
        RightShooterWheelPIDPublisher =
                create_publisher<std_msgs::msg::Float64>(right_wheel_send_topic,rclcpp::SystemDefaultsQoS());

        if(this->get_parameter("trigger_wheel_send_topic").get_type() != rclcpp::PARAMETER_STRING){
            RCLCPP_ERROR(this->get_logger(), "TYPE ERROR: \"trigger_wheel_send_topic\" must be a string.");
            return CallbackReturn::FAILURE;
        }
        trigger_wheel_send_topic = this->get_parameter("trigger_wheel_send_topic").as_string();
        TriggerWheelPIDPublisher =
                create_publisher<std_msgs::msg::Float64>(trigger_wheel_send_topic,rclcpp::SystemDefaultsQoS());


        if(this->get_parameter("shooter_wheel_receive_topic").get_type() != rclcpp::PARAMETER_STRING){
            RCLCPP_ERROR(this->get_logger(), "TYPE ERROR: \"shooter_wheel_receive_topic\" must be a string.");
            return CallbackReturn::FAILURE;
        }
        shooter_wheel_receive_topic = this->get_parameter("shooter_wheel_receive_topic").as_string();
        this->ShooterSubscription =
                this->create_subscription<std_msgs::msg::Float64>(
                        shooter_wheel_receive_topic,
                                        rclcpp::SystemDefaultsQoS(),
                                        std::bind(&ShooterController::shooter_callback,this,std::placeholders::_1));


        if(this->get_parameter("trigger_wheel_receive_topic").get_type() != rclcpp::PARAMETER_STRING){
            RCLCPP_ERROR(this->get_logger(), "TYPE ERROR: \"trigger_wheel_receive_topic\" must be a string.");
            return CallbackReturn::FAILURE;
        }
        trigger_wheel_receive_topic = this->get_parameter("trigger_wheel_receive_topic").as_string();
        this->TriggerSubscription =
                this->create_subscription<std_msgs::msg::Float64>(
                        trigger_wheel_receive_topic,
                                        rclcpp::SystemDefaultsQoS(),
                                        std::bind(&ShooterController::trigger_callback,this,std::placeholders::_1));


        if(this->get_parameter("diagnostic_topic").get_type() != rclcpp::PARAMETER_STRING){
            RCLCPP_ERROR(this->get_logger(), "TYPE ERROR: \"diagnostic_topic\" must be a string.");
            return CallbackReturn::FAILURE;
        }
        diagnostic_topic = this->get_parameter("diagnostic_topic").as_string();
        this->DiagnosticSubscription =
                this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
                        diagnostic_topic,
                                        rclcpp::SystemDefaultsQoS(),
                                        std::bind(&ShooterController::diag_callback,this,std::placeholders::_1));


        diag_objs.emplace(this->get_parameter("trigger_wheel_diag_name").as_string(),false);
        diag_objs.emplace(this->get_parameter("left_shooter_wheel_diag_name").as_string(),false);
        diag_objs.emplace(this->get_parameter("right_shooter_wheel_diag_name").as_string(),false);

        RCLCPP_INFO(this->get_logger(), "configured");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ShooterController::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        timer_update->reset();
        TriggerWheelPIDPublisher.reset();
        RightShooterWheelPIDPublisher.reset();
        LeftShooterWheelPIDPublisher.reset();
        TriggerSubscription.reset();
        ShooterSubscription.reset();

        diag_objs.clear();

        RCLCPP_INFO(this->get_logger(), "cleaned up");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ShooterController::on_activate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);
        using namespace std::chrono_literals;
        timer_update = this->create_wall_timer(1000ms/this->update_freq,[this] { data_publisher(); });
        TriggerWheelPIDPublisher->on_activate();
        RightShooterWheelPIDPublisher->on_activate();
        LeftShooterWheelPIDPublisher->on_activate();
        RCLCPP_INFO(this->get_logger(), "activated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ShooterController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        timer_update->reset();
        TriggerWheelPIDPublisher->on_deactivate();
        RightShooterWheelPIDPublisher->on_deactivate();
        LeftShooterWheelPIDPublisher->on_deactivate();
        TriggerSubscription.reset();
        ShooterSubscription.reset();

        RCLCPP_INFO(this->get_logger(), "deactivated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ShooterController::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if(timer_update.get()!=nullptr) timer_update->reset();
        if(TriggerWheelPIDPublisher.get() != nullptr) TriggerWheelPIDPublisher.reset();
        if(RightShooterWheelPIDPublisher.get() != nullptr) RightShooterWheelPIDPublisher.reset();
        if(LeftShooterWheelPIDPublisher.get() != nullptr) LeftShooterWheelPIDPublisher.reset();
        if(TriggerSubscription.get() != nullptr) TriggerSubscription.reset();
        if(ShooterSubscription.get() != nullptr) ShooterSubscription.reset();

        diag_objs.clear();

        RCLCPP_INFO(this->get_logger(), "shutdown");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ShooterController::on_error(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if(timer_update.get()!=nullptr) timer_update->reset();
        if(TriggerWheelPIDPublisher.get() != nullptr) TriggerWheelPIDPublisher.reset();
        if(RightShooterWheelPIDPublisher.get() != nullptr) RightShooterWheelPIDPublisher.reset();
        if(LeftShooterWheelPIDPublisher.get() != nullptr) LeftShooterWheelPIDPublisher.reset();
        if(TriggerSubscription.get() != nullptr) TriggerSubscription.reset();
        if(ShooterSubscription.get() != nullptr) ShooterSubscription.reset();

        diag_objs.clear();

        RCLCPP_ERROR(this->get_logger(), "Error happened");
        return CallbackReturn::SUCCESS;
    }

    void ShooterController::data_publisher() {
        static bool offline_warned = false;
        if(!motor_offline){
            if (this->trigger_on && this->shooter_on) {
                this->trigger_wheel_current_set = this->trigger_wheel_pid_target;
            } else {
                this->trigger_wheel_current_set = static_cast<double>(0.0f);
            }
            if (this->shooter_on) {
                if (this->shooter_wheel_pid_current_set < this->shooter_wheel_pid_target) {
                    this->shooter_wheel_pid_current_set += (((1000 / this->update_freq) / 5000) *
                                                            this->shooter_wheel_pid_target);
                } else {
                    this->shooter_wheel_pid_current_set = this->shooter_wheel_pid_target;
                }
            } else {
                this->shooter_wheel_pid_current_set = static_cast<double>(0.0f);
            }
            if(offline_warned){
                RCLCPP_INFO(this->get_logger(), "Motor reconnected");
            }
            offline_warned = false;
        }else{
            if(!offline_warned) {
                RCLCPP_WARN(this->get_logger(), "Shooter shut down due to motor offline.");
                offline_warned = true;
            }
            this->trigger_wheel_current_set = static_cast<double>(0.0f);
            this->shooter_wheel_pid_current_set = static_cast<double>(0.0f);
        }

        this->LeftShooterWheelPIDMsg.data = (0-shooter_wheel_pid_current_set);
        this->RightShooterWheelPIDMsg.data = shooter_wheel_pid_current_set;
        this->TriggerWheelPIDMsg.data = trigger_wheel_current_set;
        auto clock = rclcpp::Clock();
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 500, "L:%lf, R:%lf, P:%lf",this->LeftShooterWheelPIDMsg.data,
                     this->RightShooterWheelPIDMsg.data,this->TriggerWheelPIDMsg.data);

        LeftShooterWheelPIDPublisher->publish(LeftShooterWheelPIDMsg);
        RightShooterWheelPIDPublisher->publish(RightShooterWheelPIDMsg);
        TriggerWheelPIDPublisher->publish(TriggerWheelPIDMsg);
    }

    void ShooterController::shooter_callback(std_msgs::msg::Float64::SharedPtr msg) {
        if(msg->data > 0){
            this->shooter_wheel_pid_target = msg->data;
            this->shooter_on = true;
        }else if (DoubleEqual(msg->data,0.0)){
            this->shooter_on = false;
        }else{
            RCLCPP_WARN(this->get_logger(),"Received Negative settings!");
            this->shooter_on = false;
        }
    }

    void ShooterController::trigger_callback(std_msgs::msg::Float64::SharedPtr msg) {
        if(msg->data > 0){
            this->trigger_wheel_pid_target = msg->data;
            this->trigger_on = true;
        }else if (DoubleEqual(msg->data,0.0)){
            this->trigger_on = false;
        }else{
            RCLCPP_WARN(this->get_logger(),"Received Negative settings!");
            this->trigger_on = false;
        }
    }

    void ShooterController::diag_callback(diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        for (auto &i : diag_objs) {
            i.second = false;
        }
        for (const auto& status:msg->status) {
            if (status.level != status.OK){
                if(diag_objs.find(status.name) != diag_objs.end()){
                    if(!motor_offline){
                        RCLCPP_ERROR(this->get_logger(), "[%s] on status ERROR!",status.name.c_str());
                    }
                    diag_objs[status.name] = false;
                }
            } else {
                if (diag_objs.find(status.name) != diag_objs.end()){
                    diag_objs[status.name] = true;
                }
            }
        }
        bool online = true;
        for (const auto &i : diag_objs) {
            online &= i.second;
        }
        motor_offline = !online;
    }

}


int main(int argc, char const *const argv[]){
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<gary_shoot::ShooterController> shooterController =
            std::make_shared<gary_shoot::ShooterController>(rclcpp::NodeOptions());

    exe.add_node(shooterController->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_shoot::ShooterController)