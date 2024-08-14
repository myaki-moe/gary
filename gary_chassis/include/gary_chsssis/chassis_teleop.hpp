#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "gary_msgs/msg/dr16_receiver.hpp"
#include "geometry_msgs/msg/twist.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gary_chassis {
class ChassisTeleop : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit ChassisTeleop(const rclcpp::NodeOptions &options);

private:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

    void diagnostic_callback(diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
    void rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg);
    //params
    std::string diagnostic_topic;
    std::string remote_control_topic;
    std::string cmd_topic;
    double y_max_speed;
    double x_max_speed;
    double rotate_max_speed;
    gary_msgs::msg::DR16Receiver RC_control;
    geometry_msgs::msg::Twist twist;

    //publisher
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher;

    //subscriber
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_subscriber;
    rclcpp::Subscription<gary_msgs::msg::DR16Receiver>::SharedPtr rc_subscriber;

    //diagnostic message
    diagnostic_msgs::msg::DiagnosticArray diagnostic_array;
    };

}