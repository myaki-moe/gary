#include "utils/rm_motors.hpp"
#include <string>
#include <stdexcept>
#include <cmath>


using namespace utils;


RMMotor::RMMotor(MOTOR_TYPEDEF _motor_type, uint8_t _motor_id) {

    this->motor_type = _motor_type;
    this->motor_id = _motor_id;
    double rpm2rads = 2.0f * M_PI / 60.0f;

    double gear_ratio;
    double torque_constant;
    double max_current;

    switch (_motor_type) {
        case M3508: {
            //calc cmd id and feedback id
            this->feedback_id = _motor_id + 0x200;
            if (this->feedback_id >= 0x201 && this->feedback_id <= 0x204) {
                this->cmd_id = 0x200;
            } else if (this->feedback_id >= 0x205 && this->feedback_id <= 0x208) {
                this->cmd_id = 0x1FF;
            } else {
                throw std::runtime_error("invalid motor feedback id" + std::to_string(this->feedback_id));
            }

            //set motor parameters
            this->max_ecd_original = 8192;
            this->max_rpm_original = 10000;
            this->max_current_original = 16384;
            this->max_temperature_original = 125;
            this->has_temperature_sensor = true;
            gear_ratio = 3591.0f / 187.0f;
            torque_constant = 0.3f;
            max_current = 20.0f;

            break;
        }
        case M2006: {
            //calc cmd id and feedback id
            this->feedback_id = _motor_id + 0x200;
            if (this->feedback_id >= 0x201 && this->feedback_id <= 0x204) {
                this->cmd_id = 0x200;
            } else if (this->feedback_id >= 0x205 && this->feedback_id <= 0x208) {
                this->cmd_id = 0x1FF;
            } else {
                throw std::runtime_error("invalid motor feedback id" + std::to_string(this->feedback_id));
            }

            //set motor parameters
            this->max_ecd_original = 8192;
            this->max_rpm_original = 18000;
            this->max_current_original = 10000;
            this->max_temperature_original = 255;
            this->has_temperature_sensor = false;
            gear_ratio = 36.0f / 1.0f;
            torque_constant = 0.18f;
            max_current = 10.0f;

            break;
        }

        case M6020: {
            //calc cmd id and feedback id
            this->feedback_id = _motor_id + 0x204;
            if (this->feedback_id >= 0x205 && this->feedback_id <= 0x208) {
                this->cmd_id = 0x1FF;
            } else if (this->feedback_id >= 0x209 && this->feedback_id <= 0x20B) {
                this->cmd_id = 0x2FF;
            } else {
                throw std::runtime_error("invalid motor feedback id" + std::to_string(this->feedback_id));
            }

            //set motor parameters
            this->max_ecd_original = 8192;
            this->max_rpm_original = 320;
            this->max_current_original = 30000;
            this->max_temperature_original = 125;
            this->has_temperature_sensor = true;
            gear_ratio = 1.0f;
            torque_constant = 0.741f;
            max_current = 2.7f;

            break;
        }

        case M3508_GEARLESS: {
            //calc cmd id and feedback id
            this->feedback_id = _motor_id + 0x200;
            if (this->feedback_id >= 0x201 && this->feedback_id <= 0x204) {
                this->cmd_id = 0x200;
            } else if (this->feedback_id >= 0x205 && this->feedback_id <= 0x208) {
                this->cmd_id = 0x1FF;
            } else {
                throw std::runtime_error("invalid motor feedback id" + std::to_string(this->feedback_id));
            }

            //set motor parameters
            this->max_ecd_original = 8192;
            this->max_rpm_original = 15000;
            this->max_current_original = 16384;
            this->max_temperature_original = 125;
            this->has_temperature_sensor = true;
            gear_ratio = 1.0f;
            torque_constant = 0.015622f;
            max_current = 20.0f;

            break;
        }
        default: {
            throw std::runtime_error("invalid motor type");
        }
    }

    //calc cmd and feedback ratio
    this->cmd_ratio = this->max_current_original / max_current / torque_constant;
    this->position_ratio = 1.0f / gear_ratio / this->max_ecd_original * 2 * M_PI;
    this->velocity_ratio = 1.0f / gear_ratio * rpm2rads;
    this->effort_ratio = 1.0f / (double) this->max_current_original * max_current * torque_constant;

    this->feedback_data["position"] = std::make_shared<double>(0);        // rad
    this->feedback_data["encoder"] = std::make_shared<double>(0);         // rad
    this->feedback_data["encoder_raw"] = std::make_shared<double>(0);
    this->feedback_data["velocity"] = std::make_shared<double>(0);        // rad/s
    this->feedback_data["rpm"] = std::make_shared<double>(0);             // rpm
    this->feedback_data["effort"] = std::make_shared<double>(0);          // N/m
    this->feedback_data["effort_raw"] = std::make_shared<double>(0);
    if (this->has_temperature_sensor) {
        this->feedback_data["temperature"] = std::make_shared<double>(0); // C°
    }

}


bool RMMotor::cmd(double effort_set) {

    //convert effort to motor cmd
    auto cmd = static_cast<int16_t>(effort_set * this->cmd_ratio);

    bool out_of_range = false;

    //check weather cmd is in range
    if (cmd > this->max_current_original) {
        cmd = this->max_current_original;
        out_of_range = true;
    } else if (cmd < -this->max_current_original) {
        cmd = static_cast<int16_t>(-this->max_current_original);
        out_of_range = true;
    }

    this->control_cmd[0] = cmd >> 8;
    this->control_cmd[1] = cmd & 0xFF;

    return out_of_range;
}


bool RMMotor::feedback(const uint8_t fdb_data[8]) {
    //decode data
    uint16_t _ecd = ((uint16_t) (fdb_data[0] << 8 | fdb_data[1]));
    auto _rpm = static_cast<int16_t>((uint16_t) (fdb_data[2] << 8 | fdb_data[3]));
    auto _current = static_cast<int16_t>((uint16_t) (fdb_data[4] << 8 | fdb_data[5]));
    uint8_t _temperature = fdb_data[6];

    //verify data
    if (_ecd > this->max_ecd_original || (_rpm > this->max_rpm_original || _rpm < -this->max_rpm_original) ||
        (_current > this->max_current_original || _current < -this->max_current_original) ||
        _temperature > this->max_temperature_original) {
        return false;
    }

    //get position
    //skip first encoder data
    if (this->flag_first_data) {
        this->last_ecd = _ecd;
        this->flag_first_data = false;
        return true;
    }
    int delta = _ecd - this->last_ecd;
    this->last_ecd = _ecd;
    if (delta < -this->max_ecd_original / 2) delta += this->max_ecd_original;
    if (delta > this->max_ecd_original / 2) delta -= this->max_ecd_original;
    *this->feedback_data["position"] += delta * this->position_ratio;

    //get ecd
    *this->feedback_data["encoder"] = (double) _ecd * this->position_ratio;

    //get raw ecd
    *this->feedback_data["encoder_raw"] = (double) _ecd;

    //get velocity
    *this->feedback_data["velocity"] = static_cast<double>((_rpm)) * this->velocity_ratio;

    //get rpm
    *this->feedback_data["rpm"] = static_cast<double>(_rpm);

    //get effort
    *this->feedback_data["effort"] = static_cast<double>(_current) * this->effort_ratio;

    //get effort_raw
    *this->feedback_data["effort_raw"] = static_cast<double>(_current);

    //get temperature
    //if the motor has temperature sensor
    if (this->has_temperature_sensor) {
        *this->feedback_data["temperature"] = _temperature;
    }

    return true;
}
