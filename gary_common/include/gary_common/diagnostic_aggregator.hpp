#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <vector>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gary_common {

class DiagnosticAggregator : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit DiagnosticAggregator(const rclcpp::NodeOptions & options);

    private:

        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        void sub_callback(diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostic);
        void update();

        //params
        std::string diagnose_topic;
        std::string agg_topic;
        double update_freq;
        double stale_threshold;

        //publisher
        rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher;

        //subscriber
        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_subscriber;

        //timer
        rclcpp::TimerBase::SharedPtr timer_update;

        //diagnostic message
        diagnostic_msgs::msg::DiagnosticArray diagnostic_array;

        std::map<std::string, rclcpp::Time> last_update_time;
    };
}