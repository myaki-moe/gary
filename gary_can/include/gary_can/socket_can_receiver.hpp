#pragma once

#include <string>
#include <map>
#include "linux/can.h"
#include "net/if.h"

namespace driver {
namespace can {

class SocketCANReceiver {

public:
    explicit SocketCANReceiver(const std::string& _ifname);
    ~SocketCANReceiver();
    bool open_socket(uint32_t frame_id);
    bool read(uint32_t frame_id, struct can_frame *frame);
    std::string ifname;
    std::map<uint32_t, bool> is_opened;

private:
    std::map<uint32_t, int> _sockets;
};

} // namespace driver
} // namespace can