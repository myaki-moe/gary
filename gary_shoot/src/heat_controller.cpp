#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "gary_msgs/msg/dr16_receiver.hpp"
#include "std_msgs/msg/float64.hpp"
#include <string>
#include <cmath>
#include <chrono>

namespace gary_shoot{

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class HeatController : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit HeatController(const rclcpp::NodeOptions & options);


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

        void rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg);
        rclcpp::Subscription<gary_msgs::msg::DR16Receiver>::SharedPtr RemoteControlSubscription;

        void data_publisher();

        //timer
        rclcpp::TimerBase::SharedPtr timer_update;

        double update_freq;

        std::string remote_control_topic;
        std::string left_wheel_topic;
        std::string right_wheel_topic;
        double shooter_wheel_pid_target;
        double shooter_wheel_pid_current_set;
        std::string trigger_wheel_topic;
        double trigger_wheel_pid_target;
        double trigger_wheel_current_set;

        std::uint8_t prev_switch_state;
        bool shooter_on;
        bool trigger_on;
    };

    HeatController::HeatController(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode(
            "heat_controller", options) {
        this->declare_parameter("update_freq", 100.0f);
        this->declare_parameter("remote_control_topic", "/remote_control");
        this->declare_parameter("left_shooter_wheel_topic", "/fric_left_pid/cmd");
        this->declare_parameter("right_shooter_wheel_topic", "/fric_right_pid/cmd");
        this->declare_parameter("shooter_wheel_pid_target", 8500.0f);
        this->declare_parameter("trigger_wheel_topic", "/trigger_pid/cmd");
        this->declare_parameter("trigger_wheel_pid_target", 3000.0f);

        this->remote_control_topic = "/remote_control";
        this->left_wheel_topic = "/fric_left_pid/cmd";
        this->right_wheel_topic = "/fric_right_pid/cmd";
        this->shooter_wheel_pid_target = static_cast<double>(8500.0f);
        this->trigger_wheel_topic = "/trigger_pid/cmd";
        this->trigger_wheel_pid_target = static_cast<double>(3000.0f);

        this->update_freq = 100.0f;
        this->prev_switch_state = 0;
        this->shooter_on = false;
        this->trigger_on = false;
        this->shooter_wheel_pid_current_set = static_cast<double>(0.0f);
        this->trigger_wheel_current_set = static_cast<double>(0.0f);
    }

    CallbackReturn HeatController::on_configure(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if (this->get_parameter("update_freq").get_type() != rclcpp::PARAMETER_DOUBLE) {
            RCLCPP_ERROR(this->get_logger(), "update_freq type must be double");
            return CallbackReturn::FAILURE;
        }
        this->update_freq = this->get_parameter("update_freq").as_double();

        if(this->get_parameter("left_shooter_wheel_topic").get_type() != rclcpp::PARAMETER_STRING){
            RCLCPP_ERROR(this->get_logger(), "TYPE ERROR: \"left_shooter_wheel_topic\" must be a string.");
            return CallbackReturn::FAILURE;
        }
        left_wheel_topic = this->get_parameter("left_shooter_wheel_topic").as_string();
        LeftShooterWheelPIDPublisher =
                create_publisher<std_msgs::msg::Float64>(left_wheel_topic,rclcpp::SystemDefaultsQoS());

        if(this->get_parameter("right_shooter_wheel_topic").get_type() != rclcpp::PARAMETER_STRING){
            RCLCPP_ERROR(this->get_logger(), "TYPE ERROR: \"right_shooter_wheel_topic\" must be a string.");
            return CallbackReturn::FAILURE;
        }
        right_wheel_topic = this->get_parameter("right_shooter_wheel_topic").as_string();
        RightShooterWheelPIDPublisher =
                create_publisher<std_msgs::msg::Float64>(right_wheel_topic,rclcpp::SystemDefaultsQoS());

        if(this->get_parameter("trigger_wheel_topic").get_type() != rclcpp::PARAMETER_STRING){
            RCLCPP_ERROR(this->get_logger(), "TYPE ERROR: \"trigger_wheel_topic\" must be a string.");
            return CallbackReturn::FAILURE;
        }
        trigger_wheel_topic = this->get_parameter("trigger_wheel_topic").as_string();
        TriggerWheelPIDPublisher =
                create_publisher<std_msgs::msg::Float64>(trigger_wheel_topic,rclcpp::SystemDefaultsQoS());

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

        if(this->get_parameter("remote_control_topic").get_type() != rclcpp::PARAMETER_STRING){
            RCLCPP_ERROR(this->get_logger(), "TYPE ERROR: \"remote_control_topic\" must be a string.");
            return CallbackReturn::FAILURE;
        }
        remote_control_topic = this->get_parameter("remote_control_topic").as_string();

        this->RemoteControlSubscription =
                this->create_subscription<gary_msgs::msg::DR16Receiver>(
                        remote_control_topic,
                        rclcpp::SystemDefaultsQoS(),
                        std::bind(&HeatController::rc_callback,this,std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "configured");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HeatController::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        timer_update->reset();
        TriggerWheelPIDPublisher.reset();
        RightShooterWheelPIDPublisher.reset();
        LeftShooterWheelPIDPublisher.reset();
        RemoteControlSubscription.reset();

        RCLCPP_INFO(this->get_logger(), "cleaned up");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HeatController::on_activate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);
        using namespace std::chrono_literals;
        timer_update = this->create_wall_timer(1000ms/this->update_freq,[this] { data_publisher(); });
        TriggerWheelPIDPublisher->on_activate();
        RightShooterWheelPIDPublisher->on_activate();
        LeftShooterWheelPIDPublisher->on_activate();
        RCLCPP_INFO(this->get_logger(), "activated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HeatController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        timer_update->reset();
        TriggerWheelPIDPublisher->on_deactivate();
        RightShooterWheelPIDPublisher->on_deactivate();
        LeftShooterWheelPIDPublisher->on_deactivate();
        RemoteControlSubscription.reset();

        RCLCPP_INFO(this->get_logger(), "deactivated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HeatController::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if(timer_update.get()!=nullptr) timer_update->reset();
        if(TriggerWheelPIDPublisher.get() != nullptr) TriggerWheelPIDPublisher.reset();
        if(RightShooterWheelPIDPublisher.get() != nullptr) RightShooterWheelPIDPublisher.reset();
        if(LeftShooterWheelPIDPublisher.get() != nullptr) LeftShooterWheelPIDPublisher.reset();
        if(RemoteControlSubscription.get() != nullptr) RemoteControlSubscription.reset();

        RCLCPP_INFO(this->get_logger(), "shutdown");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HeatController::on_error(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if(timer_update.get()!=nullptr) timer_update->reset();
        if(TriggerWheelPIDPublisher.get() != nullptr) TriggerWheelPIDPublisher.reset();
        if(RightShooterWheelPIDPublisher.get() != nullptr) RightShooterWheelPIDPublisher.reset();
        if(LeftShooterWheelPIDPublisher.get() != nullptr) LeftShooterWheelPIDPublisher.reset();
        if(RemoteControlSubscription.get() != nullptr) RemoteControlSubscription.reset();

        RCLCPP_ERROR(this->get_logger(), "Error happened");
        return CallbackReturn::SUCCESS;
    }

    void HeatController::rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg) {
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

    void HeatController::data_publisher() {
        if(this->trigger_on && this->shooter_on){
            this->trigger_wheel_current_set = this->trigger_wheel_pid_target;
        }else{
            this->trigger_wheel_current_set = static_cast<double>(0.0f);
        }
        if(this->shooter_on){
            if(this->shooter_wheel_pid_current_set <= this->shooter_wheel_pid_target) {
                this->shooter_wheel_pid_current_set += (((1000 / this->update_freq) / 5000) *
                                                        this->shooter_wheel_pid_target);
            }else{
                this->shooter_wheel_pid_current_set = this->shooter_wheel_pid_target;
            }
        }else{
            this->shooter_wheel_pid_current_set = static_cast<double>(0.0f);
        }

        this->LeftShooterWheelPIDMsg.data = (0-shooter_wheel_pid_current_set);
        this->RightShooterWheelPIDMsg.data = shooter_wheel_pid_current_set;
        this->TriggerWheelPIDMsg.data = trigger_wheel_current_set;

        RCLCPP_DEBUG(this->get_logger(),"L:%lf, R:%lf, P:%lf",this->LeftShooterWheelPIDMsg.data,
                     this->RightShooterWheelPIDMsg.data,this->TriggerWheelPIDMsg.data);

        LeftShooterWheelPIDPublisher->publish(LeftShooterWheelPIDMsg);
        RightShooterWheelPIDPublisher->publish(RightShooterWheelPIDMsg);
        TriggerWheelPIDPublisher->publish(TriggerWheelPIDMsg);
    }
}


int main(int argc, char const *const argv[]){
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<gary_shoot::HeatController> heatController =
            std::make_shared<gary_shoot::HeatController>(rclcpp::NodeOptions());

    exe.add_node(heatController->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_shoot::HeatController)