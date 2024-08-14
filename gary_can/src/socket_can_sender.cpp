#include "gary_can/socket_can_sender.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sys/socket.h>
#include <cstring>
#include <utility>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <fcntl.h>

using namespace driver::can;


SocketCANSender::SocketCANSender(const std::string &_ifname) {
    this->_socket = 0;
    this->is_opened = false;
    this->ifname = _ifname;
}


SocketCANSender::~SocketCANSender() {
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[%s] destroying socket_can_sender", this->ifname.c_str());

    //close the socket if it is opened
    if (this->is_opened) {
        close(this->_socket);
    }
}


bool SocketCANSender::open_socket() {
    //create socket
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[%s] creating socket", this->ifname.c_str());
    this->_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    //select interface
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[%s] setting interface index",
                 this->ifname.c_str());
    struct ifreq _ifreq{};
    strcpy(_ifreq.ifr_name, this->ifname.c_str());
    if (int ret = ioctl(this->_socket, SIOCGIFINDEX, &_ifreq) != 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"),
                     "[%s] failed to select interface, return %d, error: %s",
                     this->ifname.c_str(), ret, std::strerror(errno));
        this->is_opened = false;
        close(this->_socket);
        return false;
    }

    //bind
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[%s] binding", this->ifname.c_str());
    struct sockaddr_can _sockaddr_can{};
    _sockaddr_can.can_family = AF_CAN;
    _sockaddr_can.can_ifindex = _ifreq.ifr_ifindex;
    if (int ret = ::bind(this->_socket, (struct sockaddr *) &_sockaddr_can, sizeof(_sockaddr_can)) != 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"),
                     "[%s] failed to bind to interface, return %d, error: %s",
                     this->ifname.c_str(), ret, std::strerror(errno));
        this->is_opened = false;
        close(this->_socket);
        return false;
    }

    //nonblocking mode
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[%s] setting nonblocking mode",
                 this->ifname.c_str());
    int flags = fcntl(this->_socket, F_GETFL, 0);
    if (int ret = fcntl(this->_socket, F_SETFL, flags | O_NONBLOCK) != 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"),
                     "[%s] failed to set nonblocking mode, return %d, error: %s",
                     this->ifname.c_str(), ret, std::strerror(errno));
        this->is_opened = false;
        close(this->_socket);
        return false;
    }

    //disable can filter
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[%s] disabling can filter", this->ifname.c_str());
    if (int ret = setsockopt(this->_socket, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0) != 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"),
                     "[%s] failed to disable can filter, return %d, error: %s",
                     this->ifname.c_str(), ret, std::strerror(errno));
        this->is_opened = false;
        close(this->_socket);
        return false;
    }
    this->is_opened = true;
    return true;
}


bool SocketCANSender::open_socket(const std::string &_ifname) {
    this->ifname = _ifname;
    return this->open_socket();
}


bool SocketCANSender::send(struct can_frame &tx_frame) {
    //check opened
    if (!this->is_opened) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[%s] socket is not opened",
                     this->ifname.c_str());
        return false;
    }

    //attempt to write
    long len = ::send(this->_socket, &tx_frame, sizeof(can_frame), 0);

    //check return value
    if (len == sizeof(struct can_frame)) {
        //write successfully
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[%s] send successful, id 0x%x, dlc %d",
                     this->ifname.c_str(), tx_frame.can_id, tx_frame.can_dlc);
        return true;
    } else if (len < 0) {
        if (errno == ENXIO) {
            //link is down
            RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[%s] link is down, return %d",
                         this->ifname.c_str(), len);
            this->is_opened = false;
            close(this->_socket);
        } else {
            //other error
            RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[%s] failed to write, return %d, error: %s",
                         this->ifname.c_str(), len, std::strerror(errno));
        }
    } else if (len != sizeof(struct can_frame)) {
        //write length mismatch
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"),
                     "[%s] attempt to write %d bytes, %d byte written, error: %s",
                     this->ifname.c_str(), sizeof(struct can_frame), len, std::strerror(errno));
    }
    return false;
}
