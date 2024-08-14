#include "gary_serial/dr16_receiver.hpp"
#include <cstring>
#include <chrono>
#include <regex>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>


using namespace std::chrono_literals;
using namespace gary_serial;


DR16Receiver::DR16Receiver(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode(
        "dr16_receiver", options) {

    this->declare_parameter("send_topic", "/remote_control");
    this->declare_parameter("diagnostic_topic", "/diagnostics");
    this->declare_parameter("update_freq", 100.0f);
    this->declare_parameter("diag_freq", 10.0f);
    this->declare_parameter("serial_port", "/dev/ttyDBUS0");
    this->declare_parameter("baudrate", 100000);
    this->declare_parameter("override_diag_device_name", "");

    this->update_freq = 100.0f;
    this->diag_freq = 10.0f;
    this->baudrate = 100000;
    this->fd = 0;
    this->is_opened = false;
    this->available_len = 0;
    this->decode_fail_cnt = 0;
    this->flag_transmission_jammed = false;

}

CallbackReturn DR16Receiver::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //check and create data publisher
    if (this->get_parameter("send_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "send_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->send_topic = this->get_parameter("send_topic").as_string();
    this->msg_publisher = this->create_publisher<gary_msgs::msg::DR16Receiver>(
            this->send_topic, rclcpp::SystemDefaultsQoS());

    //check and create diagnostic publisher
    if (this->get_parameter("diagnostic_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "diagnostic_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->diagnostic_topic = this->get_parameter("diagnostic_topic").as_string();
    this->diag_publisher = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            this->diagnostic_topic, rclcpp::SystemDefaultsQoS());

    //get update_freq
    if (this->get_parameter("update_freq").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "update_freq type must be double");
        return CallbackReturn::FAILURE;
    }
    this->update_freq = this->get_parameter("update_freq").as_double();

    //get diag_freq
    if (this->get_parameter("diag_freq").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "diag_freq type must be double");
        return CallbackReturn::FAILURE;
    }
    this->diag_freq = this->get_parameter("diag_freq").as_double();

    //get serial_port
    if (this->get_parameter("serial_port").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "serial_port type must be string");
        return CallbackReturn::FAILURE;
    }
    this->serial_port = this->get_parameter("serial_port").as_string();

    //get baudrate
    if (this->get_parameter("baudrate").get_type() != rclcpp::PARAMETER_INTEGER) {
        RCLCPP_ERROR(this->get_logger(), "baudrate type must be integer");
        return CallbackReturn::FAILURE;
    }
    this->baudrate = this->get_parameter("baudrate").as_int();

    //get override_diag_device_name
    if (this->get_parameter("override_diag_device_name").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "override_diag_device_name type must be string");
        return CallbackReturn::FAILURE;
    }
    this->override_diag_device_name = this->get_parameter("override_diag_device_name").as_string();

    this->dr16_msg.header.frame_id = "";
    this->diag_msg.header.frame_id = "";

    diagnostic_msgs::msg::DiagnosticStatus diagnostic_status;
    if (this->override_diag_device_name.empty()) {
        //default device name, without /dev/ prefix
        diagnostic_status.hardware_id = this->serial_port.substr(5);
        diagnostic_status.name = this->serial_port.substr(5);
    } else {
        //custom device name
        diagnostic_status.hardware_id = this->override_diag_device_name;
        diagnostic_status.name = this->override_diag_device_name;
    }
    this->diag_msg.status.emplace_back(diagnostic_status);

    RCLCPP_INFO(this->get_logger(), "configured");

    return CallbackReturn::SUCCESS;
}

CallbackReturn DR16Receiver::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //delete publisher
    this->msg_publisher.reset();
    this->diag_publisher.reset();
    this->timer_detect.reset();

    //clear diag msg
    this->diag_msg.status.clear();

    RCLCPP_INFO(this->get_logger(), "cleaning up");

    return CallbackReturn::SUCCESS;
}

CallbackReturn DR16Receiver::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //create timer
    this->timer_update = this->create_wall_timer(1000ms / this->update_freq, [this] { update(); });
    this->timer_diag = this->create_wall_timer(1000ms / this->diag_freq, [this] { publish_diag(); });
    this->timer_detect = this->create_wall_timer(1000ms, [this] { detect_jammed(); });
    //activate publisher
    this->msg_publisher->on_activate();
    this->diag_publisher->on_activate();
    //open serial
    this->open();
    //update timestamp
    this->last_update_timestamp = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "activated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn DR16Receiver::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //delete timer
    this->timer_update.reset();
    this->timer_diag.reset();
    this->timer_detect.reset();
    //deactivate publisher
    this->msg_publisher->on_deactivate();
    this->diag_publisher->on_deactivate();
    //close serial
    this->close();

    RCLCPP_INFO(this->get_logger(), "deactivated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn DR16Receiver::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    if (this->msg_publisher.get() != nullptr) this->msg_publisher.reset();
    if (this->diag_publisher.get() != nullptr) this->diag_publisher.reset();
    if (this->timer_update.get() != nullptr) this->timer_update.reset();
    if (this->timer_diag.get() != nullptr) this->timer_diag.reset();
    if (this->timer_detect.get() != nullptr) this->timer_detect.reset();
    this->close();

    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}

CallbackReturn DR16Receiver::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    if (this->msg_publisher.get() != nullptr) this->msg_publisher.reset();
    if (this->diag_publisher.get() != nullptr) this->diag_publisher.reset();
    if (this->timer_update.get() != nullptr) this->timer_update.reset();
    if (this->timer_diag.get() != nullptr) this->timer_diag.reset();
    if (this->timer_detect.get() != nullptr) this->timer_detect.reset();
    this->close();

    RCLCPP_INFO(this->get_logger(), "error");
    return CallbackReturn::SUCCESS;
}

bool DR16Receiver::open() {
    int ret;

    if (this->is_opened) this->close();

    this->fd = ::open(this->serial_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (this->fd == -1) return false;

    //get param
    struct termios2 options{};
    ret = ioctl(fd, TCGETS2, &options);
    if (ret < 0) return false;

    //custom baudrate
    options.c_cflag &= ~CBAUD;
    options.c_cflag |= BOTHER;
    options.c_ispeed = this->baudrate;
    options.c_ospeed = this->baudrate;
    //8 data bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    //even parity
    options.c_cflag |= PARENB;
    options.c_cflag &= ~PARODD;
    //1 stop bit
    options.c_cflag &= ~CSTOPB;

    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~IGNBRK;
    /* set input mode (non−canonical, no echo,...) */
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;

    options.c_oflag = 0;                  // no remapping, no delays
    options.c_cflag |= (CLOCAL | CREAD);  // ignore modem controls, enable reading

    //set param
    ret = ioctl(fd, TCSETS2, &options);
    if (ret < 0) return false;

    is_opened = true;
    return true;
}

void DR16Receiver::close() {
    if (this->is_opened) ::close(this->fd);
    this->is_opened = false;
}

bool DR16Receiver::get_available_len() {

    //check serial open
    if (!is_opened) return false;

    int ret;
    int len;

    //get available len
    ret = ioctl(this->fd, FIONREAD, &len, sizeof(len));

    //failed to get
    if (ret < 0) {
        RCLCPP_DEBUG(this->get_logger(), "failed to get buffer available length");
        return false;
    }

    //check serial disconnect
    if (errno == 5) {
        RCLCPP_DEBUG(this->get_logger(), "serial port disconnect");
        errno = 0;
        return false;
    }

    this->available_len = len;
    return true;
}

bool DR16Receiver::read() {

    //check serial open
    if (!is_opened) {
        if (!this->open()) {
            RCLCPP_DEBUG(this->get_logger(), "failed to reopen serial port");
            return false;
        }
    }

    //get available length
    if (!this->get_available_len()) {
        this->close();
        RCLCPP_DEBUG(this->get_logger(), "failed to get available length");
        return false;
    }

    //too many available data
    if (this->available_len > 18 * 2) {
        RCLCPP_DEBUG(this->get_logger(), "too many data in buffer %d", this->available_len);
        while (::read(this->fd, this->buff, 18) > 0) {
            RCLCPP_DEBUG(this->get_logger(), "flushing");
        }
        return false;
    }

    //available length less than a packet
    if (this->available_len < 18) {

        //sleep 1ms and retry
        RCLCPP_DEBUG(this->get_logger(), "sleep 1ms and retry");
        rclcpp::sleep_for(1ms);

        if (!this->get_available_len()) {
            this->close();
            RCLCPP_DEBUG(this->get_logger(), "failed to get available length");
            return false;
        }
        //still less than a packet
        if (this->available_len < 18) {
            RCLCPP_DEBUG(this->get_logger(), "waiting 1ms but still less than packet %d", this->available_len);
            return false;
        }
    }

    long n = ::read(this->fd, this->buff, 18);
    if (n == 18) {
        RCLCPP_DEBUG(this->get_logger(), "read 1 packet succ");
        //clear buffer
        ::read(this->fd, this->buff, 18);
        return true;
    } else {
        RCLCPP_DEBUG(this->get_logger(), "read length mismatch");
        return false;
    }
}

bool DR16Receiver::decode() {
    int32_t ch0, ch1, ch2, ch3;
    uint8_t s1, s2;
    int32_t x, y, z;
    uint8_t l, r;
    uint16_t key;
    int32_t wheel;

    //joystick channel
    ch0 = ((int16_t) this->buff[0] | ((int16_t) this->buff[1] << 8)) & 0x07FF;
    ch0 -= 1024;
    ch1 = (((int16_t) this->buff[1] >> 3) | ((int16_t) this->buff[2] << 5)) & 0x07FF;
    ch1 -= 1024;
    ch2 = (((int16_t) this->buff[2] >> 6) | ((int16_t) this->buff[3] << 2) | ((int16_t) this->buff[4] << 10)) & 0x07FF;
    ch2 -= 1024;
    ch3 = (((int16_t) this->buff[4] >> 1) | ((int16_t) this->buff[5] << 7)) & 0x07FF;
    ch3 -= 1024;
    //switch
    s1 = ((this->buff[5] >> 4) & 0x000C) >> 2;
    s2 = ((this->buff[5] >> 4) & 0x0003);
    //mouse
    x = ((int16_t) this->buff[6]) | ((int16_t) this->buff[7] << 8);
    y = ((int16_t) this->buff[8]) | ((int16_t) this->buff[9] << 8);
    z = ((int16_t) this->buff[10]) | ((int16_t) this->buff[11] << 8);
    l = this->buff[12];
    r = this->buff[13];
    //keyboard
    key = this->buff[14] | this->buff[15] << 8;
    //wheel
    wheel = ((int16_t) this->buff[16]) | ((int16_t) this->buff[17] << 8);
    wheel -= 1024;

    //validate range
    if (abs(ch0) > 660 || abs(ch1) > 660 || abs(ch2) > 660 || abs(ch3) > 660 || s1 < 1 || s1 > 3 || s2 < 1 ||
        s2 > 3 || l > 1 || l < 0 || r > 1 || r < 0 || abs(wheel) > 660) {
        RCLCPP_DEBUG(this->get_logger(), "failed to decode");
        this->decode_fail_cnt++;
        return false;
    }

    this->dr16_msg.ch_right_x = static_cast<float>(ch0) / 660;
    this->dr16_msg.ch_right_y = static_cast<float>(ch1) / 660;
    this->dr16_msg.ch_left_x = static_cast<float>(ch2) / 660;
    this->dr16_msg.ch_left_y = static_cast<float>(ch3) / 660;
    this->dr16_msg.sw_left = s1;
    this->dr16_msg.sw_right = s2;
    this->dr16_msg.mouse_x = static_cast<float>(x);
    this->dr16_msg.mouse_y = static_cast<float>(y);
    this->dr16_msg.mouse_z = static_cast<float>(z);
    this->dr16_msg.mouse_press_l = l;
    this->dr16_msg.mouse_press_r = r;
    this->dr16_msg.ch_wheel = static_cast<float>(wheel) / 660;

    this->dr16_msg.key_w = (key & 0x01) != 0;
    this->dr16_msg.key_s = (key & 0x02) != 0;
    this->dr16_msg.key_a = (key & 0x04) != 0;
    this->dr16_msg.key_d = (key & 0x08) != 0;
    this->dr16_msg.key_shift = (key & 0x10) != 0;
    this->dr16_msg.key_ctrl = (key & 0x20) != 0;
    this->dr16_msg.key_q = (key & 0x40) != 0;
    this->dr16_msg.key_e = (key & 0x80) != 0;
    this->dr16_msg.key_r = ((key >> 8) & 0x01) != 0;
    this->dr16_msg.key_f = ((key >> 8) & 0x02) != 0;
    this->dr16_msg.key_g = ((key >> 8) & 0x04) != 0;
    this->dr16_msg.key_z = ((key >> 8) & 0x08) != 0;
    this->dr16_msg.key_x = ((key >> 8) & 0x10) != 0;
    this->dr16_msg.key_c = ((key >> 8) & 0x20) != 0;
    this->dr16_msg.key_v = ((key >> 8) & 0x40) != 0;
    this->dr16_msg.key_b = ((key >> 8) & 0x80) != 0;

    RCLCPP_DEBUG(this->get_logger(), "decode succ");
    return true;
}


void DR16Receiver::publish_data() {
    this->dr16_msg.header.stamp = this->get_clock()->now();
    this->msg_publisher->publish(this->dr16_msg);
}

void DR16Receiver::update() {

    RCLCPP_DEBUG(this->get_logger(), "update");


    if (!this->read()) {
        RCLCPP_DEBUG(this->get_logger(), "read() failed");
        return;
    }
    if (!this->decode()) {
        RCLCPP_DEBUG(this->get_logger(), "decode() failed");
        return;
    }

    this->publish_data();

    //update timestamp
    this->last_update_timestamp = this->get_clock()->now();

}

void DR16Receiver::publish_diag() {
    this->diag_msg.header.stamp = this->get_clock()->now();
    if (!this->is_opened) {

        this->diag_msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        this->diag_msg.status[0].message = "serial device offline";

        rclcpp::Clock clock;
        RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, 1000, "[%s] serial device offline",
                              this->diag_msg.status[0].name.c_str());

    } else if (this->get_clock()->now() - this->last_update_timestamp > 500ms) {

        this->diag_msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        this->diag_msg.status[0].message = "receiver offline";

        rclcpp::Clock clock;
        RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, 1000, "[%s] receiver offline",
                              this->diag_msg.status[0].name.c_str());

    } else if (this->flag_transmission_jammed) {

        this->diag_msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        this->diag_msg.status[0].message = "transmission jammed";

        rclcpp::Clock clock;
        RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "[%s] transmission jammed",
                             this->diag_msg.status[0].name.c_str());

    } else {
        this->diag_msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        this->diag_msg.status[0].message = "ok";
    }
    this->diag_publisher->publish(this->diag_msg);
}

void DR16Receiver::detect_jammed() {
    //10 packet decode failed can be considered as transmission jammed
    if (this->decode_fail_cnt > 10) {
        this->flag_transmission_jammed = true;
    } else {
        this->flag_transmission_jammed = false;
    }
    this->decode_fail_cnt = 0;
}

int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<DR16Receiver> dr16_receiver = std::make_shared<DR16Receiver>(rclcpp::NodeOptions());

    exe.add_node(dr16_receiver->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_serial::DR16Receiver)