#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <vector>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gary_can {

struct can_recv_info_t {
    std::string device;
    int can_id;
    int matches;
};

class SocketCANMonitor : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit SocketCANMonitor(const rclcpp::NodeOptions & options);

    private:

        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        std::vector<struct can_recv_info_t> update_rcvlist(const std::string& path);
        bool open_socket(const std::string& ifname);

        void update();

        //params
        std::string diagnose_topic;
        double update_freq;
        std::vector<std::string> monitored_can_bus;
        double overload_threshold;

        //publisher
        rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher;

        //timer
        rclcpp::TimerBase::SharedPtr timer_update;

        //diagnostic message
        diagnostic_msgs::msg::DiagnosticArray diagnostic_array;

        std::vector<struct can_recv_info_t> rcvlist_all;
        std::vector<struct can_recv_info_t> rcvlist_fil;

        std::map<std::string, int> last_recv_cnt;
        std::map<std::string, std::map<int, int>> last_filter_cnt;
    };
}
