
#include "gary_chsssis/chassis_teleop.hpp"

using namespace std::chrono_literals;
using namespace gary_chassis;


ChassisTeleop::ChassisTeleop(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("chassis_teleop",
                                                                                                   options) {

    this->declare_parameter("cmd_topic");
    this->declare_parameter("diagnostic_topic", "/diagnostics_agg");
    this->declare_parameter("remote_control", "/remote_control");
    this->declare_parameter("y_max_speed");
    this->declare_parameter("x_max_speed");
    this->declare_parameter("rotate_max_speed");
}

CallbackReturn ChassisTeleop::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);


    //get diagnostic_topic
    if (this->get_parameter("diagnostic_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "diagnostic_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->diagnostic_topic = this->get_parameter("diagnostic_topic").as_string();
    this->diagnostic_subscriber = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            this->diagnostic_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ChassisTeleop::diagnostic_callback, this, std::placeholders::_1));

    //get topic_cmd
    if (this->get_parameter("cmd_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "cmd_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->cmd_topic = this->get_parameter("cmd_topic").as_string();
    this->cmd_publisher = this->create_publisher<geometry_msgs::msg::Twist>(this->cmd_topic,
                                                                            rclcpp::SystemDefaultsQoS());
    //get remote_control
    if (this->get_parameter("remote_control").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "remote_control type must be string");
        return CallbackReturn::FAILURE;
    }
    this->remote_control_topic = this->get_parameter("remote_control").as_string();
    this->rc_subscriber = this->create_subscription<gary_msgs::msg::DR16Receiver>(
            this->remote_control_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ChassisTeleop::rc_callback, this, std::placeholders::_1));

    //get y_max_speed
    if (this->get_parameter("y_max_speed").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "y_max_speed type must be double");
        return CallbackReturn::FAILURE;
    }
    this->y_max_speed = this->get_parameter("y_max_speed").as_double();
    //get x_max_speed
    if (this->get_parameter("x_max_speed").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "x_max_speed type must be double");
        return CallbackReturn::FAILURE;
    }
    this->x_max_speed = this->get_parameter("x_max_speed").as_double();

    //get rotate_max_speed
    if (this->get_parameter("rotate_max_speed").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "rotate_max_speed type must be double");
        return CallbackReturn::FAILURE;
    }
    this->rotate_max_speed = this->get_parameter("rotate_max_speed").as_double();


    RCLCPP_INFO(this->get_logger(), "configured");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisTeleop::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    this->cmd_publisher.reset();
    this->diagnostic_subscriber.reset();
    this->rc_subscriber.reset();
    RCLCPP_INFO(this->get_logger(), "cleaning up");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisTeleop::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);
    this->cmd_publisher->on_activate();
    RCLCPP_INFO(this->get_logger(), "activated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisTeleop::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);
    this->cmd_publisher->on_deactivate();
    RCLCPP_INFO(this->get_logger(), "deactivated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisTeleop::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);
    if (this->cmd_publisher.get() != nullptr) this->cmd_publisher.reset();
    if (this->diagnostic_subscriber.get() != nullptr) this->diagnostic_subscriber.reset();
    if (this->rc_subscriber.get() != nullptr) this->rc_subscriber.reset();

    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisTeleop::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    if (this->cmd_publisher.get() != nullptr) this->cmd_publisher.reset();
    if (this->diagnostic_subscriber.get() != nullptr) this->diagnostic_subscriber.reset();
    if (this->rc_subscriber.get() != nullptr) this->rc_subscriber.reset();

    RCLCPP_INFO(this->get_logger(), "error");
    return CallbackReturn::SUCCESS;
}

void ChassisTeleop::rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg) {
    if (!this->cmd_publisher->is_activated()) return;
    RC_control = *msg;
    if (RC_control.sw_right == gary_msgs::msg::DR16Receiver::SW_DOWN) {
        return;
    } else if (RC_control.sw_right == gary_msgs::msg::DR16Receiver::SW_MID) {
        twist.linear.x = RC_control.ch_left_y * x_max_speed;
        twist.linear.y = -RC_control.ch_left_x * y_max_speed;
        twist.angular.z = -RC_control.ch_wheel * rotate_max_speed;
    }

    std::map<std::string, double> chassis_speed;
    chassis_speed.insert(std::make_pair("vx", twist.linear.x));
    chassis_speed.insert(std::make_pair("vy", twist.linear.y));
    chassis_speed.insert(std::make_pair("az", twist.angular.z));
    cmd_publisher->publish(twist);

}

void ChassisTeleop::diagnostic_callback(diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    //store the data
    this->diagnostic_array = *msg;
}

int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr <ChassisTeleop> chassis_teleop = std::make_shared<ChassisTeleop>(rclcpp::NodeOptions());

    exe.add_node(chassis_teleop->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_chassis::ChassisTeleop)