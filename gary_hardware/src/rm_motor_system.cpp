#include "gary_hardware/rm_motor_system.hpp"

#include <chrono>
#include <string>
#include <memory>
#include <vector>

namespace gary_hardware {

    hardware_interface::return_type RMMotorSystem::configure(const hardware_interface::HardwareInfo &info) {
        //get system name
        this->system_name = info.name;

        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "configuring");

        //call the base class initializer
        if (configure_default(info) != hardware_interface::return_type::OK) {
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }

        //check parameter "can_bus"
        if (info.hardware_parameters.count("can_bus") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger(this->system_name), "invalid can bus definition in urdf");
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
        std::string bus_name = info.hardware_parameters.at("can_bus");
        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "using can bus %s", bus_name.c_str());

        //create socket can sender and receiver
        this->can_sender = std::make_shared<driver::can::SocketCANSender>(bus_name);
        this->can_receiver = std::make_shared<driver::can::SocketCANReceiver>(bus_name);

        //check parameter "cmd_id"
        if (info.hardware_parameters.count("cmd_id") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger(this->system_name), "invalid cmd id definition in urdf");
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
        this->cmd_id = std::stoi(info.hardware_parameters.at("cmd_id"), nullptr, 16);
        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "using cmd id 0x%x", cmd_id);

        RCLCPP_INFO(rclcpp::get_logger(this->system_name), "configuring %d motors", info.joints.size());

        //foreach motor
        for (const auto &i: info.joints) {
            std::string motor_name = i.name;
            //check parameter "motor_id"
            if (i.parameters.count("motor_id") != 1) {
                RCLCPP_ERROR(rclcpp::get_logger(this->system_name), "invalid motor id in urdf");
                this->status_ = hardware_interface::status::UNKNOWN;
                return hardware_interface::return_type::ERROR;
            }
            int motor_id = std::stoi(i.parameters.at("motor_id"));

            //check parameter "motor_type"
            if (i.parameters.count("motor_type") != 1) {
                RCLCPP_ERROR(rclcpp::get_logger(this->system_name), "invalid motor type in urdf");
                this->status_ = hardware_interface::status::UNKNOWN;
                return hardware_interface::return_type::ERROR;
            }
            std::string motor_type = i.parameters.at("motor_type");

            //check parameter "update_rate"
            int update_rate = 1000;
            if (i.parameters.count("update_rate") == 1) {
                update_rate = std::stoi(i.parameters.at("update_rate"));
                RCLCPP_INFO(rclcpp::get_logger(this->system_name),
                            "[motor name %s id %d type %s] using custom update rate %d",
                            motor_name.c_str(), motor_id, motor_type.c_str(), update_rate);
            }
            //calculate offline detection threshold
            double threshold = 1.0f / (double) update_rate;
            if (threshold < 0.1f) threshold = 0.1f;
            if (threshold > 1.0f) threshold = 1.0f;

            //create new motor
            std::shared_ptr<utils::RMMotor> new_motor;

            //switch motor type
            if (motor_type == "m3508") {
                new_motor = std::make_shared<utils::RMMotor>(utils::M3508, motor_id);
            } else if (motor_type == "m2006") {
                new_motor = std::make_shared<utils::RMMotor>(utils::M2006, motor_id);
            } else if (motor_type == "m6020") {
                new_motor = std::make_shared<utils::RMMotor>(utils::M6020, motor_id);
            } else if (motor_type == "m3508_gearless") {
                new_motor = std::make_shared<utils::RMMotor>(utils::M3508_GEARLESS, motor_id);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger(this->system_name), "invalid motor type");
                this->status_ = hardware_interface::status::UNKNOWN;
                return hardware_interface::return_type::ERROR;
            }

            //check if motor id and cmd id is mismatched
            if (new_motor->cmd_id != this->cmd_id) {
                RCLCPP_ERROR(rclcpp::get_logger(this->system_name), "motor cmd id mismatched");
                this->status_ = hardware_interface::status::UNKNOWN;
                return hardware_interface::return_type::ERROR;
            }

            //make motor, name, cmd and offline detector in pairs
            rm_motor_ctrl_t motor_ctrl;
            motor_ctrl.motor = new_motor;
            motor_ctrl.cmd = std::make_shared<double>(0);
            motor_ctrl.cmd_raw = std::make_shared<double>(0);
            motor_ctrl.motor_name = motor_name;
            motor_ctrl.offlineDetector = std::make_shared<utils::OfflineDetector>(threshold);
            motor_ctrl.offline = std::make_shared<double>(0);

            //bind feedback can id
            if (!this->can_receiver->open_socket(new_motor->feedback_id)) {
                RCLCPP_ERROR(rclcpp::get_logger(this->system_name), "[%s] failed to bind can id 0x%x to bus",
                             this->can_receiver->ifname.c_str(), new_motor->feedback_id);
            }

            //add to motors
            this->motors.emplace_back(motor_ctrl);

            RCLCPP_INFO(rclcpp::get_logger(this->system_name),
                        "add new motor, name: %s, type: %s, can bus: %s, cmd id: 0x%x, feedback id: 0x%x",
                        motor_name.c_str(), motor_type.c_str(), bus_name.c_str(), new_motor->cmd_id, new_motor->feedback_id);
        }

        //open can sender
        if (!this->can_sender->open_socket()) {
            RCLCPP_ERROR(rclcpp::get_logger(this->system_name), "[%s] failed to open can sender socket",
                         this->can_sender->ifname.c_str());
        }

        this->status_ = hardware_interface::status::CONFIGURED;
        RCLCPP_INFO(rclcpp::get_logger(this->system_name), "configured");
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> RMMotorSystem::export_state_interfaces() {

        //creat state interfaces
        std::vector<hardware_interface::StateInterface> state_interfaces;

        //foreach motor and add state interfaces
        for (const auto &i: this->motors) {
            auto motor_name = i.motor_name;
            auto motor_obj = i.motor;
            //foreach motor feedback data
            for (const auto &j: motor_obj->feedback_data) {
                auto feedback_name = j.first;
                auto feedback_data = j.second;
                state_interfaces.emplace_back(motor_name, feedback_name, feedback_data.get());
            }
            state_interfaces.emplace_back(motor_name, "offline", i.offline.get());
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RMMotorSystem::export_command_interfaces() {

        //creat state interfaces
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        //foreach motor and add command interfaces
        for (const auto &i: this->motors) {
            auto motor_name = i.motor_name;
            auto cmd = i.cmd.get();
            auto cmd_raw = i.cmd_raw.get();
            command_interfaces.emplace_back(motor_name, "effort", cmd);
            command_interfaces.emplace_back(motor_name, "raw", cmd_raw);
        }

        return command_interfaces;
    }

    hardware_interface::return_type RMMotorSystem::start() {

        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "starting");

        //foreach all motor
        for (const auto &i: this->motors) {
            //initialize cmd with zero
            *i.cmd = 0;
            *i.cmd_raw = 0;
            //update offline detector
            i.offlineDetector->update(true);
        }

        this->status_ = hardware_interface::status::STARTED;

        RCLCPP_INFO(rclcpp::get_logger(this->system_name), "started");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RMMotorSystem::stop() {

        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "stopping");

        this->status_ = hardware_interface::status::STOPPED;

        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "stopped");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RMMotorSystem::read() {

        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "reading");

        //foreach motor and read the corresponding can data
        for (const auto &i: this->motors) {

            //update offline status
            *i.offline = i.offlineDetector->offline;
            if (i.offlineDetector->offline) {
                rclcpp::Clock clock;
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(this->system_name), clock, 1000, "[%s] offline",
                                     i.motor_name.c_str());
            }

            //check if socket is down
            if (!this->can_receiver->is_opened[i.motor->feedback_id]) {
                //reopen socket
                if (!this->can_receiver->open_socket(i.motor->feedback_id)) {
                    //reopen failed
                    rclcpp::Clock clock;
                    RCLCPP_WARN_THROTTLE(rclcpp::get_logger(this->system_name), clock, 1000,
                                         "[%s] can receiver reopen failed, id 0x%x",
                                         this->can_receiver->ifname.c_str(), i.motor->feedback_id);
                    i.offlineDetector->update(false);
                    continue;
                }
            }

            struct can_frame can_recv_frame{};
            //attempt to read feedback

            //get the latest data, read until socket buffer is empty
            bool succ = false;
            while (true) {
                struct can_frame can_recv_frame_temp{};
                if (this->can_receiver->read(i.motor->feedback_id, &can_recv_frame_temp)) {
                    can_recv_frame = can_recv_frame_temp;
                    succ |= true;
                } else {
                    succ |= false;
                    break;
                }
            }

            if (succ) {
                RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "[%s] can frame read succ",
                             this->can_receiver->ifname.c_str());
                //decode
                if (i.motor->feedback(can_recv_frame.data)) {
                    //read success
                    i.offlineDetector->update(true);
                } else {
                    RCLCPP_DEBUG(rclcpp::get_logger(this->system_name),
                                 "[%s] motor data decode failed, name: %s, id: 0x%x",
                                 this->can_receiver->ifname.c_str(), i.motor_name.c_str(), i.motor->feedback_id);
                    i.offlineDetector->update(false);
                }
            } else {
                RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "[%s] can read failed id: 0x%X",
                             this->can_receiver->ifname.c_str(), i.motor->feedback_id);
                i.offlineDetector->update(false);
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RMMotorSystem::write() {

        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "writing");

        //check if socket is down
        if (!this->can_sender->is_opened) {
            //reopen socket
            if (!this->can_sender->open_socket()) {
                rclcpp::Clock clock;
                RCLCPP_WARN_THROTTLE(rclcpp::get_logger(this->system_name), clock, 1000,
                                     "[%s] can sender reopen failed",
                                     this->can_sender->ifname.c_str());
                return hardware_interface::return_type::ERROR;
            }
        }

        struct can_frame frame{};
        frame.can_dlc = 8;
        frame.can_id = this->cmd_id;

        //foreach motor and fill the can cmd
        for (const auto &i: this->motors) {
            //encode motor command
            uint8_t motor_cmd[2];

            if (*i.cmd != 0) {
                i.motor->cmd(*i.cmd);
                motor_cmd[0] = i.motor->control_cmd[0];
                motor_cmd[1] = i.motor->control_cmd[1];
            } else {
                motor_cmd[0] = static_cast<int16_t>(*i.cmd_raw) >> 8;
                motor_cmd[1] = static_cast<int16_t>(*i.cmd_raw) & 0xFF;
            }

            //get motor id
            int id = i.motor->motor_id;
            if (id >= 5) id -= 4;

            //fill the cmd in a can frame
            frame.data[(id - 1) * 2 + 0] = motor_cmd[0];
            frame.data[(id - 1) * 2 + 1] = motor_cmd[1];

            //always clean the command to keep safe
            *i.cmd = 0;
            *i.cmd_raw = 0;
        }

        //send the can frame
        if (this->can_sender->send(frame)) {
            RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "[%s] can frame send succ, cmd: 0x%x",
                         this->can_sender->ifname.c_str(), this->cmd_id);
            return hardware_interface::return_type::OK;
        } else {
            rclcpp::Clock clock;
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger(this->system_name), clock, 1000,
                                 "[%s] can send failed, cmd: 0x%x",
                                 this->can_sender->ifname.c_str(), this->cmd_id);
        }
        return hardware_interface::return_type::ERROR;
    }

}   //namespace gary_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gary_hardware::RMMotorSystem, hardware_interface::SystemInterface)
