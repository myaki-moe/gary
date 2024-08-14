#pragma once

#include <string>
#include <functional>
#include "linux/can.h"
#include "net/if.h"

namespace driver {
namespace can {

class SocketCANSender {

public:
    explicit SocketCANSender(const std::string& _ifname);
    ~SocketCANSender();
    bool open_socket();
    bool open_socket(const std::string& _ifname);
    bool send(struct can_frame& tx_frame);
    std::string ifname;
    bool is_opened;

private:
    int _socket;
};

} // namespace driver
} // namespace can