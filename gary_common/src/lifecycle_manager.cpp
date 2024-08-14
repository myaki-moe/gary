#include "gary_common/lifecycle_manager.hpp"


using namespace std::chrono_literals;
using namespace gary_common;


LifecycleManager::LifecycleManager(const rclcpp::NodeOptions & options) : rclcpp::Node(
        "lifecycle_manager", options) {


    this->declare_parameter("node_names", std::vector<std::string>());
    this->declare_parameter("diagnose_topic", "/diagnostics");
    this->declare_parameter("diag_freq", 10.0f);
    this->declare_parameter("respawn", true);
    this->declare_parameter("update_rate", 10.0f);

    //get node_names
    if (this->get_parameter("node_names").get_type() != rclcpp::PARAMETER_STRING_ARRAY) {
        RCLCPP_ERROR(this->get_logger(), "node_names type must be string array");
        return;
    }
    this->node_names = this->get_parameter("node_names").as_string_array();

    //get diagnose_topic
    if (this->get_parameter("diagnose_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "diagnose_topic type must be string");
        return;
    }
    this->diagnose_topic = this->get_parameter("diagnose_topic").as_string();

    //get diag_freq
    if (this->get_parameter("diag_freq").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "diag_freq type must be float");
        return;
    }
    this->diag_freq = this->get_parameter("diag_freq").as_double();

    //get respawn
    if (this->get_parameter("respawn").get_type() != rclcpp::PARAMETER_BOOL) {
        RCLCPP_ERROR(this->get_logger(), "respawn type must be bool");
        return;
    }
    this->respawn = this->get_parameter("respawn").as_bool();

    //get update_rate
    if (this->get_parameter("update_rate").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "update_rate type must be float");
        return;
    }
    this->update_rate = this->get_parameter("update_rate").as_double();

    //create service client for all nodes
    for (const auto& node_name : this->node_names) {
        auto change_state_client = this->create_client<lifecycle_msgs::srv::ChangeState>(node_name + "/change_state");
        auto get_state_client = this->create_client<lifecycle_msgs::srv::GetState>(node_name + "/get_state");

        this->change_state_clients.emplace(node_name, change_state_client);
        this->get_state_clients.emplace(node_name, get_state_client);
        this->node_states.emplace(node_name, "offline");
        this->change_state_configure_flag.emplace(node_name, false);
        this->change_state_activate_flag.emplace(node_name, false);
    }

    //publisher
    this->diag_publisher = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(this->diagnose_topic, rclcpp::SystemDefaultsQoS());
    this->diag_msg.header.frame_id = "";

    //timers
    this->timer_update = this->create_wall_timer(1000ms / this->update_rate, std::bind(&LifecycleManager::timer_update_callback, this));
    this->timer_diag = this->create_wall_timer(1000ms / this->diag_freq, std::bind(&LifecycleManager::timer_diagnose_callback, this));

    //param callback
    this->param_callback_handle = this->add_on_set_parameters_callback(std::bind(&LifecycleManager::param_update_callback, this, std::placeholders::_1));
}


void LifecycleManager::timer_update_callback() {

    RCLCPP_DEBUG(this->get_logger(), "timer callback");

    //foreach all monitored nodes
    for(auto& node_name : this->node_names) {

        //check service status
        if (! this->get_state_clients[node_name]->service_is_ready() || ! this->change_state_clients[node_name]->service_is_ready()) {
            rclcpp::Clock clock;
            RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, 1000, "[%s] node offline", node_name.c_str());
            this->node_states[node_name] = "node offline";
        } else {
            //check last get node status request status
            if (this->get_state_responses.count(node_name) == 0) {
                //open a new request if it is the first time
                RCLCPP_DEBUG(this->get_logger(), "[%s] get_state service is not sent yet", node_name.c_str());
                auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
                auto response = get_state_clients[node_name]->async_send_request(request);
                this->get_state_responses[node_name] = response;
            } else {
                //check if last request is finished
                if (this->get_state_responses[node_name].wait_for(0ms) == std::future_status::ready) {
                    //send new request
                    this->node_states[node_name] = this->get_state_responses[node_name].get()->current_state.label;
                    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
                    auto response = get_state_clients[node_name]->async_send_request(request);
                    this->get_state_responses[node_name] = response;
                }
            }
            //check last set node status request status
            if (this->change_state_responses.count(node_name) == 0) {
                //change node state to configure
                if (this->node_states[node_name] == "unconfigured") {
                    //reconfigure the node if respawn is set
                    if (!this->change_state_configure_flag[node_name] || this->respawn) {
                        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
                        request->transition.label = "configure";
                        auto response = change_state_clients[node_name]->async_send_request(request);
                        this->change_state_responses[node_name] = response;
                        this->change_state_configure_flag[node_name] = true;
                        RCLCPP_INFO(this->get_logger(), "[%s] changing to configure", node_name.c_str());
                    }
                } else if (this->node_states[node_name] == "inactive") {
                    if (!this->change_state_activate_flag[node_name] || this->respawn) {
                        //reconfigure the node if respawn is set
                        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
                        request->transition.label = "activate";
                        auto response = change_state_clients[node_name]->async_send_request(request);
                        this->change_state_responses[node_name] = response;
                        this->change_state_activate_flag[node_name] = true;
                        RCLCPP_INFO(this->get_logger(), "[%s] changing to activate", node_name.c_str());
                    }
                }
            } else {
                //wait last request finished
                auto status = this->change_state_responses[node_name].wait_for(0ms);
                if (status == std::future_status::ready) {
                    RCLCPP_DEBUG(this->get_logger(), "[%s] changing state succ", node_name.c_str());
                    this->change_state_responses.erase(node_name);
                } else if(status == std::future_status::timeout) {
                    RCLCPP_DEBUG(this->get_logger(), "[%s] changing state timeout", node_name.c_str());
                    this->change_state_responses.erase(node_name);
                } else if(status == std::future_status::deferred) {
                    RCLCPP_DEBUG(this->get_logger(), "[%s] changing state sent, deferred", node_name.c_str());
                }
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "[%s] %s", node_name.c_str(), this->node_states[node_name].c_str());
    }
}


void LifecycleManager::timer_diagnose_callback() {
    RCLCPP_DEBUG(this->get_logger(), "timer diagnose callback");

    this->diag_msg.status.clear();
    for(const auto& i : this->node_states) {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = i.first;
        status.hardware_id = i.first;
        status.level = (i.second == "node offline") ? diagnostic_msgs::msg::DiagnosticStatus::ERROR : diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = i.second;
        this->diag_msg.status.emplace_back(status);
    }

    //add self
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = this->get_name();
    status.hardware_id = this->get_name();
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "active";
    this->diag_msg.status.emplace_back(status);

    this->diag_msg.header.stamp = this->get_clock()->now();
    this->diag_publisher->publish(this->diag_msg);
}


rcl_interfaces::msg::SetParametersResult LifecycleManager::param_update_callback(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;

    for (const auto& param : params) {
        if (param.get_name() == "node_names") {
            //get node_names
            if (param.get_type() != rclcpp::PARAMETER_STRING_ARRAY) {
                RCLCPP_ERROR(this->get_logger(), "node_names type must be string array");
                result.successful = false;
                result.reason = "node_names type must be string array";
                return result;
            }
            std::vector<std::string> old_node_names = this->node_names;
            this->node_names = param.as_string_array();
            //create client for new node
            for (const auto& node_name : this->node_names) {

                if (std::count(old_node_names.begin(), old_node_names.end(), node_name) != 0) continue;

                auto change_state_client = this->create_client<lifecycle_msgs::srv::ChangeState>(node_name + "/change_state");
                auto get_state_client = this->create_client<lifecycle_msgs::srv::GetState>(node_name + "/get_state");

                this->change_state_clients.emplace(node_name, change_state_client);
                this->get_state_clients.emplace(node_name, get_state_client);
                this->node_states.emplace(node_name, "node offline");
                this->change_state_configure_flag.emplace(node_name, false);
                this->change_state_activate_flag.emplace(node_name, false);
            }

        } else if (param.get_name() == "diagnose_topic") {
            //get diagnose_topic
            if (param.get_type() != rclcpp::PARAMETER_STRING) {
                RCLCPP_ERROR(this->get_logger(), "diagnose_topic type must be string");
                result.successful = false;
                result.reason = "diagnose_topic type must be string";
                return result;
            }
            //create new publisher
            this->diagnose_topic = param.as_string();
            this->diag_publisher.reset();
            this->diag_publisher = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(this->diagnose_topic, rclcpp::SystemDefaultsQoS());

        } else if (param.get_name() == "diag_freq") {
            //get diag_freq
            if (param.get_type() != rclcpp::PARAMETER_DOUBLE) {
                RCLCPP_ERROR(this->get_logger(), "diag_freq type must be double");
                result.successful = false;
                result.reason = "diag_freq type must be double";
                return result;
            }
            //creat new timer
            this->diag_freq = param.as_double();
            this->timer_diag->reset();
            this->timer_diag = this->create_wall_timer(1000ms / this->diag_freq, std::bind(&LifecycleManager::timer_diagnose_callback, this));

        } else if (param.get_name() == "respawn") {
            //get diag_freq
            if (param.get_type() != rclcpp::PARAMETER_BOOL) {
                RCLCPP_ERROR(this->get_logger(), "respawn type must be bool");
                result.successful = false;
                result.reason = "respawn type must be bool";
                return result;
            }
            this->respawn = param.as_bool();

        } else if (param.get_name() == "update_rate") {
            //get update_rate
            if (param.get_type() != rclcpp::PARAMETER_DOUBLE) {
                RCLCPP_ERROR(this->get_logger(), "update_rate type must be double");
                result.successful = false;
                result.reason = "update_rate type must be double";
                return result;
            }
            //creat new timer
            this->update_rate = param.as_double();
            this->timer_update->reset();
            this->timer_update = this->create_wall_timer(1000ms / this->update_rate, std::bind(&LifecycleManager::timer_update_callback, this));
        }
    }
    RCLCPP_INFO(this->get_logger(), "update param succ");
    result.successful = true;
    result.reason = "ok";
    return result;
}

int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<LifecycleManager> lifecycle_manager = std::make_shared<LifecycleManager>(rclcpp::NodeOptions());

    exe.add_node(lifecycle_manager);

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_common::LifecycleManager)