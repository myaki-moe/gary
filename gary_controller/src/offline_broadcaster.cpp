#include "gary_controller/offline_broadcaster.hpp"


using namespace gary_controller;


OfflineBroadcaster::OfflineBroadcaster() : pub_rate(10.0f), publisher(), flag_publish(false) {}

controller_interface::return_type OfflineBroadcaster::init(const std::string &controller_name) {

    //call the base class initializer
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK) return ret;

    this->auto_declare("interface_name", "offline");
    this->auto_declare("diagnose_topic", "/diagnostics");
    this->auto_declare("pub_rate", 10.0f);

    return controller_interface::return_type::OK;
}


controller_interface::InterfaceConfiguration OfflineBroadcaster::state_interface_configuration() const {

    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;

    return state_interfaces_config;
}


controller_interface::InterfaceConfiguration OfflineBroadcaster::command_interface_configuration() const {

    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;

    return command_interfaces_config;
}


CallbackReturn OfflineBroadcaster::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    RCLCPP_DEBUG(this->get_node()->get_logger(), "configuring");

    //get parameter: interface_name
    if (this->get_node()->get_parameter("interface_name").get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_node()->get_logger(), "interface_name type must be string");
        return CallbackReturn::ERROR;
    }
    this->interface_name = this->get_node()->get_parameter("interface_name").as_string();

    //get parameter: diagnose_topic
    if (this->get_node()->get_parameter("diagnose_topic").get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_node()->get_logger(), "diagnose_topic type must be string");
        return CallbackReturn::ERROR;
    }
    this->diagnose_topic = this->get_node()->get_parameter("diagnose_topic").as_string();

    //get parameter: pub_rate
    if (this->get_node()->get_parameter("pub_rate").get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_node()->get_logger(), "pub_rate type must be double");
        return CallbackReturn::ERROR;
    }
    this->pub_rate = this->get_node()->get_parameter("pub_rate").as_double();

    //create publisher
    auto publisher_ = this->get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(this->diagnose_topic,
                                                                                                rclcpp::SystemDefaultsQoS());
    this->publisher = std::make_unique<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(publisher_);
    this->publisher->unlock();

    RCLCPP_INFO(this->get_node()->get_logger(), "configured");
    return CallbackReturn::SUCCESS;
}


CallbackReturn OfflineBroadcaster::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    RCLCPP_DEBUG(this->get_node()->get_logger(), "activating");

    this->last_time = this->get_node()->get_clock()->now();

    RCLCPP_INFO(this->get_node()->get_logger(), "activated");

    return CallbackReturn::SUCCESS;
}


CallbackReturn OfflineBroadcaster::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    RCLCPP_DEBUG(this->get_node()->get_logger(), "deactivating");


    RCLCPP_INFO(this->get_node()->get_logger(), "deactivated");

    return CallbackReturn::SUCCESS;
}


controller_interface::return_type OfflineBroadcaster::update() {
    RCLCPP_DEBUG(this->get_node()->get_logger(), "updating");

    if (this->flag_publish) {
        if(this->publisher->trylock()) {
            //clear the previous data
            this->publisher->msg_.status.clear();
            //foreach all state interfaces
            for(const auto &i:this->state_interfaces_) {
                if (i.get_interface_name() == this->interface_name) {
                    auto diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
                    diagnostic_status.hardware_id = i.get_name();
                    diagnostic_status.name = i.get_name();
                    if (i.get_value() == 1) {
                        diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                        diagnostic_status.message = "offline";
                        RCLCPP_DEBUG(this->get_node()->get_logger(), "[%s] offline", i.get_name().c_str(), i.get_value());
                    } else {
                        diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                        diagnostic_status.message = "ok";
                        RCLCPP_DEBUG(this->get_node()->get_logger(), "[%s] ok", i.get_name().c_str(), i.get_value());
                    }
                    this->publisher->msg_.status.emplace_back(diagnostic_status);
                }
            }
            this->publisher->msg_.header.frame_id = "";
            this->publisher->msg_.header.stamp = this->get_node()->get_clock()->now();
            this->publisher->unlockAndPublish();
            this->flag_publish = false;
        }
    }

    //get current time
    auto time_now = this->get_node()->get_clock()->now();

    //control publish rate
    if (time_now - this->last_time < (1000ms / this->pub_rate)) {
        return controller_interface::return_type::OK;
    }

    this->flag_publish = true;
    this->last_time = time_now;
    return controller_interface::return_type::OK;
}


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gary_controller::OfflineBroadcaster, controller_interface::ControllerInterface)