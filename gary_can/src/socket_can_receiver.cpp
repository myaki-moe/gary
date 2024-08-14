#include "gary_can/socket_can_receiver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sys/socket.h>
#include <cstring>
#include <utility>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <fcntl.h>

using namespace driver::can;


SocketCANReceiver::SocketCANReceiver(const std::string &_ifname) {
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "creating socket_can_receiver");
    this->ifname = _ifname;
}


SocketCANReceiver::~SocketCANReceiver() {
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "destroying socket_can_receiver");

    //close all opened sockets
    for (auto i: this->_sockets) {
        if (this->is_opened[i.first]) {
            RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[socket %d] closing socket for id 0x%x",
                         i.second, i.first);
            close(i.second);
        }
    }
}


bool SocketCANReceiver::open_socket(uint32_t frame_id) {

    //create socket
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[%s] creating socket for id 0x%x",
                 this->ifname.c_str(), frame_id);
    int _socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);


    //select interface
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[%s] setting interface index for id: 0x%x",
                 this->ifname.c_str(), frame_id);
    struct ifreq _ifreq{};
    strcpy(_ifreq.ifr_name, this->ifname.c_str());
    if (int ret = ioctl(_socket, SIOCGIFINDEX, &_ifreq) != 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"),
                     "[%s] failed to select interface for id 0x%x, return %d, error: %s",
                     this->ifname.c_str(), frame_id, ret, std::strerror(errno));
        close(_socket);
        return false;
    }

    //bind
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[%s] binding for id 0x%x",
                 this->ifname.c_str(), frame_id);
    struct sockaddr_can _sockaddr_can{};
    _sockaddr_can.can_family = AF_CAN;
    _sockaddr_can.can_ifindex = _ifreq.ifr_ifindex;
    if (int ret = ::bind(_socket, (struct sockaddr *) &_sockaddr_can, sizeof(_sockaddr_can)) != 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"),
                     "[%s] failed to bind to interface for id 0x%x, return %d, error: %s",
                     this->ifname.c_str(), frame_id, ret, std::strerror(errno));
        close(_socket);
        return false;
    }

    //set filter
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[%s] setting can filter for id 0x%x",
                 this->ifname.c_str(), frame_id);
    struct can_filter _can_filter[1];
    _can_filter[0].can_id = frame_id;
    _can_filter[0].can_mask = CAN_SFF_MASK;
    if (int ret = setsockopt(_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &_can_filter, sizeof(_can_filter)) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("socket_can_receiver"),
                     "[%s] failed to set can filter for id 0x%x, return %d, error: %s",
                     this->ifname.c_str(), frame_id, ret, std::strerror(errno));
        close(_socket);
        return false;
    }

    //nonblocking mode
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[%s] setting nonblocking mode for id 0x%x",
                 this->ifname.c_str(), frame_id);
    int flags = fcntl(_socket, F_GETFL, 0);
    if (int ret = fcntl(_socket, F_SETFL, flags | O_NONBLOCK) != 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"),
                     "[%s] failed to set nonblocking mode for id 0x%x, return %d, error: %s",
                     this->ifname.c_str(), frame_id, ret, std::strerror(errno));
        close(_socket);
        return false;
    }

    //a socket only receive one frame id, make them a pair
    this->_sockets[frame_id] = _socket;
    this->is_opened[frame_id] = true;

    return true;
}


bool SocketCANReceiver::read(uint32_t frame_id, struct can_frame *frame) {

    //check frame id is valid
    if (this->_sockets.count(frame_id) == 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[%s] unbinded id 0x%x",
                     this->ifname.c_str(), frame_id);
        return false;
    }

    //check socket is opened
    if (!this->is_opened[frame_id]) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[%s] socket is closed, id 0x%x",
                     this->ifname.c_str(), frame_id);
        return false;
    }

    //attempt to read
    long len = ::recv(this->_sockets[frame_id], frame, sizeof(can_frame), 0);

    //check return value
    if (len == sizeof(can_frame)) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[%s] read successful, id 0x%x, dlc %d",
                     this->ifname.c_str(), frame_id, frame->can_dlc);
        return true;
    } else if (len < 0) {
        if (errno == ENODEV) {
            //link is down
            RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[%s] link is down, id 0x%x, return %d",
                         this->ifname.c_str(), frame_id, len);
            this->is_opened[frame_id] = false;
            close(this->_sockets[frame_id]);
        } else {
            RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"),
                         "[%s] failed to read, id 0x%x, return %d, error: %s",
                         this->ifname.c_str(), frame_id, len, std::strerror(errno));
        }
    } else if (len != sizeof(struct can_frame)) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"),
                     "[%s] attempt to read %d bytes, %d byte read, error: %s",
                     this->ifname.c_str(), errno, sizeof(struct can_frame), std::strerror(len));
    }
    return false;
}
