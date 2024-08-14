#include "gary_can/socket_can_monitor.hpp"
#include <sys/socket.h>
#include <cstring>
#include <linux/can/raw.h>
#include <linux/can.h>
#include <net/if.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <libsocketcan.h>
#include <sys/ioctl.h>
#include <regex>
#include <fstream>

using namespace std::chrono_literals;
using namespace gary_can;


SocketCANMonitor::SocketCANMonitor(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode(
        "socket_can_monitor", options) {

    this->declare_parameter("diagnose_topic", "/diagnostics");
    this->declare_parameter("update_freq", 10.0f);
    this->declare_parameter("monitored_can_bus", rclcpp::ParameterValue(std::vector<std::string>()));
    this->declare_parameter("overload_threshold", 0.8f);

    this->update_freq = 10.0f;
    this->overload_threshold = 0.8f;
}

CallbackReturn SocketCANMonitor::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //check and create publisher
    if (this->get_parameter("diagnose_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "diagnose_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->diagnose_topic = this->get_parameter("diagnose_topic").as_string();
    this->diagnostic_publisher = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            this->diagnose_topic, rclcpp::SystemDefaultsQoS());

    //get update_freq
    if (this->get_parameter("update_freq").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "update_freq type must be double");
        return CallbackReturn::FAILURE;
    }
    this->update_freq = this->get_parameter("update_freq").as_double();

    //get monitored_can_bus
    if (this->get_parameter("monitored_can_bus").get_type() != rclcpp::PARAMETER_STRING_ARRAY) {
        RCLCPP_ERROR(this->get_logger(), "monitored_can_bus type must be string array");
        return CallbackReturn::FAILURE;
    }
    this->monitored_can_bus = this->get_parameter("monitored_can_bus").as_string_array();
    if (this->monitored_can_bus.empty()) RCLCPP_WARN(this->get_logger(), "no can bus is monitored");

    //get overload_threshold
    if (this->get_parameter("overload_threshold").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "overload_threshold type must be double");
        return CallbackReturn::FAILURE;
    }
    this->overload_threshold = this->get_parameter("overload_threshold").as_double();

    RCLCPP_INFO(this->get_logger(), "configured");

    return CallbackReturn::SUCCESS;
}

CallbackReturn SocketCANMonitor::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //delete publisher
    this->diagnostic_publisher.reset();

    RCLCPP_INFO(this->get_logger(), "cleaning up");

    return CallbackReturn::SUCCESS;
}

CallbackReturn SocketCANMonitor::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //create timer
    this->timer_update = this->create_wall_timer(1000ms / this->update_freq, [this] { update(); });
    //activate publisher
    this->diagnostic_publisher->on_activate();

    //create socket
    for (const auto &i: this->monitored_can_bus) {
        this->open_socket(i);
    }

    RCLCPP_INFO(this->get_logger(), "activated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn SocketCANMonitor::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //delete timer
    this->timer_update.reset();
    //deactivate publisher
    this->diagnostic_publisher->on_deactivate();

    RCLCPP_INFO(this->get_logger(), "deactivated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn SocketCANMonitor::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    if (this->diagnostic_publisher.get() != nullptr) this->diagnostic_publisher.reset();
    if (this->timer_update.get() != nullptr) this->timer_update.reset();

    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}

CallbackReturn SocketCANMonitor::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    if (this->diagnostic_publisher.get() != nullptr) this->diagnostic_publisher.reset();
    if (this->timer_update.get() != nullptr) this->timer_update.reset();

    RCLCPP_INFO(this->get_logger(), "error");
    return CallbackReturn::SUCCESS;
}

std::vector<struct can_recv_info_t> SocketCANMonitor::update_rcvlist(const std::string &path) {
    std::ifstream infile;
    std::string str;
    std::regex base_regex(R"(^ +(.+?) +([\d\w]+) +([\d\w]+) +([\d\w]+) +([\d\w]+) +([\d\w]+) +([\d\w]+))");
    std::smatch base_match;
    std::vector<struct can_recv_info_t> rcvlist;
    struct can_recv_info_t can_recv_info;

    RCLCPP_DEBUG(this->get_logger(), "reading rcvlist %s", path.c_str());

    infile.open(path);

    while (true) {
        if (infile.eof()) break;

        std::getline(infile, str);

        if (std::regex_match(str, base_match, base_regex)) {

            //skip title
            if (base_match[1].str() == "device") continue;

            can_recv_info.device = base_match[1].str();
            can_recv_info.can_id = std::stoi(base_match[2].str());
            can_recv_info.matches = std::stoi(base_match[6].str());
            rcvlist.emplace_back(can_recv_info);
        }
    }
    infile.close();

    RCLCPP_DEBUG(this->get_logger(), "total item %d", rcvlist.size());

    return rcvlist;
}

bool SocketCANMonitor::open_socket(const std::string &ifname) {
    //create new socket
    RCLCPP_DEBUG(this->get_logger(), "creating socket");
    int _socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    //select interface
    RCLCPP_DEBUG(this->get_logger(), "[%s] setting interface index", ifname.c_str());
    struct ifreq _ifreq{};
    strcpy(_ifreq.ifr_name, ifname.c_str());
    if (int ret = ioctl(_socket, SIOCGIFINDEX, &_ifreq) != 0) {
        RCLCPP_DEBUG(this->get_logger(), "[%s] failed to select interface, return %d, error: %s",
                     ifname.c_str(), ret, std::strerror(errno));
        close(_socket);
        return false;
    }

    //bind
    RCLCPP_DEBUG(this->get_logger(), "binding");
    struct sockaddr_can _sockaddr_can{};
    _sockaddr_can.can_family = AF_CAN;
    _sockaddr_can.can_ifindex = _ifreq.ifr_ifindex;
    if (int ret = ::bind(_socket, (struct sockaddr *) &_sockaddr_can, sizeof(_sockaddr_can)) != 0) {
        RCLCPP_DEBUG(this->get_logger(), "failed to bind to interface, return %d, error: %s",
                     ret, std::strerror(errno));
        close(_socket);
        return false;
    }

    //set filter
    RCLCPP_DEBUG(this->get_logger(), "setting can filter");
    struct can_filter _can_filter[1];
    _can_filter[0].can_id = 0;
    _can_filter[0].can_mask = 0;
    if (int ret = setsockopt(_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &_can_filter, sizeof(_can_filter)) != 0) {
        RCLCPP_DEBUG(this->get_logger(), "failed to set can filter, return %d, error: %s",
                     ret, std::strerror(errno));
        close(_socket);
        return false;
    }

    //nonblocking mode
    RCLCPP_DEBUG(this->get_logger(), "setting nonblocking mode");
    int flags = fcntl(_socket, F_GETFL, 0);
    if (int ret = fcntl(_socket, F_SETFL, flags | O_NONBLOCK) != 0) {
        RCLCPP_DEBUG(this->get_logger(), "failed to set nonblocking mode, return %d, error: %s",
                     ret, std::strerror(errno));
        close(_socket);
        return false;
    }

    RCLCPP_DEBUG(this->get_logger(), "create socket successful");
    return true;
}

void SocketCANMonitor::update() {

    RCLCPP_DEBUG(this->get_logger(), "update");

    //monitored nothing
    if (this->monitored_can_bus.empty()) return;

    this->diagnostic_array.status.clear();
    this->rcvlist_all = this->update_rcvlist("/proc/net/can/rcvlist_all");
    this->rcvlist_fil = this->update_rcvlist("/proc/net/can/rcvlist_fil");

    //foreach all ifindex
    for (const auto &i: this->monitored_can_bus) {

        diagnostic_msgs::msg::DiagnosticStatus diagnostic_status;
        diagnostic_status.hardware_id = i;
        diagnostic_status.name = i;

        //check existence and get delta pkt
        int delta_pkt = 0;
        bool found = false;
        for (const auto &j: this->rcvlist_all) {
            if (j.device == i) {
                delta_pkt = j.matches - this->last_recv_cnt[i];
                found = true;
                this->last_recv_cnt[i] = j.matches;
            }
        }
        if (!found) {
            this->last_recv_cnt[i] = 0;
            //reopen socket
            this->open_socket(i);

            diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            diagnostic_status.message = "offline";
            diagnostic_array.status.emplace_back(diagnostic_status);
            auto clock = rclcpp::Clock();
            RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, 1000, "[%s] offline", i.c_str());
            continue;
        }

        //check if transmission jammed
        int state;
        can_get_state(i.c_str(), &state);
        if (state > CAN_STATE_ERROR_ACTIVE) {
            diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diagnostic_status.message = "transmission jammed";
            diagnostic_array.status.emplace_back(diagnostic_status);
            auto clock = rclcpp::Clock();
            RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "[%s] transmission jammed, state %d", i.c_str(),
                                 state);
            continue;
        }

        //check if overloaded
        //get bitrate
        struct can_bittiming bittiming{};
        if (can_get_bittiming(i.c_str(), &bittiming) != 0) {
            //get bitrate failed
            auto clock = rclcpp::Clock();
            RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "[%s] failed to get bitrate", i.c_str());
            diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diagnostic_status.message = "failed to get bitrate";
            diagnostic_array.status.emplace_back(diagnostic_status);
            continue;
        }

        //calc load rate
        int packet_len = 110;
        double load = (static_cast<double>(delta_pkt) * packet_len) / (bittiming.bitrate / this->update_freq);

        //check
        if (load > this->overload_threshold) {
            diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diagnostic_status.message = "can bus overload";
            diagnostic_array.status.emplace_back(diagnostic_status);
            auto clock = rclcpp::Clock();
            RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "[%s] can bus overload %f", i.c_str(), load);
            continue;
        }

        //no error detected
        diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        diagnostic_status.message = "ok";

        RCLCPP_DEBUG(this->get_logger(), "[%s] delta pkt %ld, bitrate %d, load %f state %d", i.c_str(), delta_pkt,
                                     bittiming.bitrate, load, state);

        //report load rate
        diagnostic_msgs::msg::KeyValue bus_load;
        bus_load.key = "bus_load";
        bus_load.value = std::to_string(load);
        diagnostic_status.values.emplace_back(bus_load);

        //report filter
        for (const auto& j : this->rcvlist_fil) {
            if (j.device == i) {
                diagnostic_msgs::msg::KeyValue filter_cnt;
                delta_pkt = j.matches - this->last_filter_cnt[i][j.can_id];
                this->last_filter_cnt[i][j.can_id] = j.matches;
                filter_cnt.key = "id_" + std::to_string(j.can_id) + "_freq";
                filter_cnt.value = std::to_string(delta_pkt * this->update_freq);
                diagnostic_status.values.emplace_back(filter_cnt);
            }
        }

        diagnostic_array.status.emplace_back(diagnostic_status);
    }

    //publish
    diagnostic_array.header.frame_id = "";
    diagnostic_array.header.stamp = this->get_clock()->now();
    this->diagnostic_publisher->publish(diagnostic_array);
}

int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<SocketCANMonitor> socket_can_monitor = std::make_shared<SocketCANMonitor>(rclcpp::NodeOptions());

    exe.add_node(socket_can_monitor->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_can::SocketCANMonitor)