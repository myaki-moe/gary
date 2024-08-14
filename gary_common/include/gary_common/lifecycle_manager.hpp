#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <vector>

namespace gary_common {

class LifecycleManager : public rclcpp::Node {

    public:
        explicit LifecycleManager(const rclcpp::NodeOptions & options);

    private:

        //params
        std::vector<std::string> node_names;
        std::string diagnose_topic;
        double diag_freq;
        bool respawn;
        double update_rate;

        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_publisher;

        rclcpp::TimerBase::SharedPtr timer_update;
        rclcpp::TimerBase::SharedPtr timer_diag;

        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

        diagnostic_msgs::msg::DiagnosticArray diag_msg;

        std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> change_state_clients;
        std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> get_state_clients;
        std::map<std::string, rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr> trans_event_subscribers;
        std::map<std::string, std::shared_future<lifecycle_msgs::srv::GetState::Response::SharedPtr>> get_state_responses;
        std::map<std::string, std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr>> change_state_responses;
        std::map<std::string, bool> change_state_configure_flag;
        std::map<std::string, bool> change_state_activate_flag;
        std::map<std::string, std::string> node_states;

        void timer_update_callback();
        void timer_diagnose_callback();
        rcl_interfaces::msg::SetParametersResult param_update_callback(const std::vector<rclcpp::Parameter>& params);
    };
}
