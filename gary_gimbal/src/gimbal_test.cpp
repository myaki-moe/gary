#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

class GimbalTest : public rclcpp::Node {

public:
    explicit GimbalTest(const rclcpp::NodeOptions & options) : Node("gimbal_test", options) {
        this->joint_subscription = this->create_subscription<control_msgs::msg::DynamicJointState>("/dynamic_joint_states", rclcpp::SystemDefaultsQoS(), std::bind(&GimbalTest::joint_callback,
                                                                                                                                                                   this, std::placeholders::_1));
        this->imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>("/gimbal_imu_broadcaster/imu", rclcpp::SystemDefaultsQoS(), std::bind(&GimbalTest::imu_callback, this, std::placeholders::_1));
    }


private:
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;
    sensor_msgs::msg::Imu imu;

    void joint_callback(control_msgs::msg::DynamicJointState::SharedPtr joint_state) {
        for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
            if (joint_state->joint_names[i] == "gimbal_yaw") {
                for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j) {
                    if (joint_state->interface_values[i].interface_names[j] == "encoder") {
                        double origin = joint_state->interface_values[i].values[j];
                        double fixed = origin;
                        fixed = origin - 1.66;
                        if (fixed < 0) fixed+=6.28;

                        double euler_x = atan2(2 * (imu.orientation.y*imu.orientation.z + imu.orientation.w*imu.orientation.x), imu.orientation.w*imu.orientation.w - imu.orientation.x*imu.orientation.x - imu.orientation.y*imu.orientation.y + imu.orientation.z*imu.orientation.z);

                        RCLCPP_INFO(this->get_logger(), "origin %f fixed %f yaw %f", origin, fixed, euler_x);
                    }
                }
            }
        }
    }

    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg) {
        this->imu = *msg;
    }
};

int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<GimbalTest> gimbal_test = std::make_shared<GimbalTest>(rclcpp::NodeOptions());

    exe.add_node(gimbal_test->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}