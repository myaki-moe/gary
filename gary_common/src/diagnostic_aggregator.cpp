#include "gary_common/diagnostic_aggregator.hpp"
#include <cstring>
#include <chrono>
#include <regex>
#include <fstream>

using namespace std::chrono_literals;
using namespace gary_common;


DiagnosticAggregator::DiagnosticAggregator(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode(
        "diagnostic_aggregator", options) {

    this->declare_parameter("diagnose_topic", "/diagnostics");
    this->declare_parameter("agg_topic", "/diagnostics_agg");
    this->declare_parameter("update_freq", 10.0f);
    this->declare_parameter("stale_threshold", 0.5f);

    this->update_freq = 10.0f;
    this->stale_threshold = 0.5f;
}

CallbackReturn DiagnosticAggregator::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //check and create publisher
    if (this->get_parameter("agg_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "agg_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->agg_topic = this->get_parameter("agg_topic").as_string();
    this->diagnostic_publisher = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            this->agg_topic, rclcpp::SystemDefaultsQoS());

    //check and create subscriber
    if (this->get_parameter("diagnose_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "diagnose_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->diagnose_topic = this->get_parameter("diagnose_topic").as_string();
    this->diagnostic_subscriber = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            this->diagnose_topic, rclcpp::SystemDefaultsQoS(), std::bind(&DiagnosticAggregator::sub_callback, this, std::placeholders::_1));

    //get update_freq
    if (this->get_parameter("update_freq").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "update_freq type must be double");
        return CallbackReturn::FAILURE;
    }
    this->update_freq = this->get_parameter("update_freq").as_double();

    //get stale_threshold
    if (this->get_parameter("stale_threshold").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "stale_threshold type must be double");
        return CallbackReturn::FAILURE;
    }
    this->stale_threshold = this->get_parameter("stale_threshold").as_double();

    RCLCPP_INFO(this->get_logger(), "configured");

    return CallbackReturn::SUCCESS;
}

CallbackReturn DiagnosticAggregator::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //delete publisher
    this->diagnostic_publisher.reset();

    //delete subscriber
    this->diagnostic_subscriber.reset();

    RCLCPP_INFO(this->get_logger(), "cleaning up");

    return CallbackReturn::SUCCESS;
}

CallbackReturn DiagnosticAggregator::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //create timer
    this->timer_update = this->create_wall_timer(1000ms / this->update_freq, [this] { update(); });
    //activate publisher
    this->diagnostic_publisher->on_activate();

    RCLCPP_INFO(this->get_logger(), "activated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn DiagnosticAggregator::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //delete timer
    this->timer_update.reset();
    //deactivate publisher
    this->diagnostic_publisher->on_deactivate();

    RCLCPP_INFO(this->get_logger(), "deactivated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn DiagnosticAggregator::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    if (this->diagnostic_publisher.get() != nullptr) this->diagnostic_publisher.reset();
    if (this->diagnostic_subscriber.get() != nullptr) this->diagnostic_subscriber.reset();
    if (this->timer_update.get() != nullptr) this->timer_update.reset();

    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}

CallbackReturn DiagnosticAggregator::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    if (this->diagnostic_publisher.get() != nullptr) this->diagnostic_publisher.reset();
    if (this->diagnostic_subscriber.get() != nullptr) this->diagnostic_subscriber.reset();
    if (this->timer_update.get() != nullptr) this->timer_update.reset();

    RCLCPP_INFO(this->get_logger(), "error");
    return CallbackReturn::SUCCESS;
}

void DiagnosticAggregator::sub_callback(diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostic) {

    RCLCPP_DEBUG(this->get_logger(), "sub_callback");

    for (const auto& i : diagnostic->status) {
        RCLCPP_DEBUG(this->get_logger(), "[%s] level %s", i.hardware_id.c_str(), i.message.c_str());
        bool match = false;
        for(auto& j: this->diagnostic_array.status) {
            if (i.hardware_id == j.hardware_id) {
                j.name = i.name;
                j.level = i.level;
                j.message = i.message;
                j.values = i.values;
                this->last_update_time[i.hardware_id] = this->get_clock()->now();
                match = true;
                break;
            }
        }
        if (!match) {
            this->diagnostic_array.status.emplace_back(i);
            this->last_update_time[i.hardware_id] = this->get_clock()->now();
        }
    }
}

void DiagnosticAggregator::update() {

    RCLCPP_DEBUG(this->get_logger(), "update");

    this->diagnostic_array.header.frame_id = "";
    this->diagnostic_array.header.stamp = this->get_clock()->now();

    auto time = this->get_clock()->now();

    for(auto &i : this->diagnostic_array.status) {
        auto delta = time - this->last_update_time[i.hardware_id];
        if (delta.seconds() > this->stale_threshold) {
            i.level = diagnostic_msgs::msg::DiagnosticStatus_<std::allocator<void>>::STALE;
            i.message = "stale";
            rclcpp::Clock clock;
            RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "[%s] stale", i.hardware_id.c_str());
        }
    }

    this->diagnostic_publisher->publish(this->diagnostic_array);
}

int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<DiagnosticAggregator> diagnostic_aggregator = std::make_shared<DiagnosticAggregator>(rclcpp::NodeOptions());

    exe.add_node(diagnostic_aggregator->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_common::DiagnosticAggregator)