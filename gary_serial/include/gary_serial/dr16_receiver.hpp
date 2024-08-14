#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "gary_msgs/msg/dr16_receiver.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <vector>
#include <chrono>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gary_serial {

class DR16Receiver : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit DR16Receiver(const rclcpp::NodeOptions & options);

    private:

        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        bool open();
        void close();

        bool get_available_len();
        bool read();
        bool decode();
        void publish_data();

        void update();

        void publish_diag();

        void detect_jammed();

        //params
        std::string send_topic;
        std::string diagnostic_topic;
        double update_freq;
        double diag_freq;
        std::string serial_port;
        int64_t baudrate;
        std::string override_diag_device_name;

        //publisher
        rclcpp_lifecycle::LifecyclePublisher<gary_msgs::msg::DR16Receiver>::SharedPtr msg_publisher;
        rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_publisher;

        //timer
        rclcpp::TimerBase::SharedPtr timer_update;
        rclcpp::TimerBase::SharedPtr timer_diag;
        rclcpp::TimerBase::SharedPtr timer_detect;

        //message
        gary_msgs::msg::DR16Receiver dr16_msg;
        diagnostic_msgs::msg::DiagnosticArray diag_msg;

        int fd;
        bool is_opened;
        int available_len;
        int decode_fail_cnt;
        bool flag_transmission_jammed;
        uint8_t buff[18]{};
        rclcpp::Time last_update_timestamp;
    };
}