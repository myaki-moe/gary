#pragma once

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "gary_can/socket_can_receiver.hpp"
#include "gary_can/socket_can_sender.hpp"
#include "utils/offline_detector.hpp"

namespace gary_hardware{
    class RMIMUSensor :public hardware_interface::BaseInterface<hardware_interface::SensorInterface>
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(RMIMUSensor)

        hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        hardware_interface::return_type start() override;

        hardware_interface::return_type stop() override;

        hardware_interface::return_type read() override;

    private:
        std::shared_ptr<driver::can::SocketCANReceiver> can_receiver;
        std::shared_ptr<utils::OfflineDetector> offlineDetector;
        std::string sensor_name;
        int can_ids[3]{};
        double sensor_data[16]{};
        double offline{};
    };
}
