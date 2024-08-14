#include "utils/mecanum_kinematics.hpp"

using namespace gary_chassis;

MecanumKinematics::MecanumKinematics(double a, double b, double r) {
    this->a = a;
    this->b = b;
    this->r = r;
}

std::map<std::string, double> MecanumKinematics::forward_solve(const std::map<std::string, double>& wheel_speed,
                                                               wheel_offline_e wheel_offline) const {
    std::map<std::string, double> chassis_speed;

    double lf = wheel_speed.at("left_front");
    double lb = wheel_speed.at("left_back");
    double rf = wheel_speed.at("right_front");
    double rb = wheel_speed.at("right_back");

    double vx = (-rf + lf + lb - rb) / 4 * this->r;
    double vy = (-rf - lf + lb + rb) / 4 * this->r;
    double az = (-rf - lf - lb - rb) / 4 / (this->a + this->b);

    chassis_speed.insert(std::make_pair("vx", vx));
    chassis_speed.insert(std::make_pair("vy", vy));
    chassis_speed.insert(std::make_pair("az", az));

    return chassis_speed;
}

std::map<std::string, double> MecanumKinematics::inverse_solve(const std::map<std::string, double>& chassis_speed,
                                                               wheel_offline_e wheel_offline) const {
    std::map<std::string, double> wheel_speed;

    double vx = chassis_speed.at("vx");
    double vy = chassis_speed.at("vy");
    double az = chassis_speed.at("az");

    if (wheel_offline == WHEEL_OFFLINE_NONE) {
        double rf = (- vx - vy + az * (a + b)) / this->r;
        double lf = (+ vx - vy + az * (a + b)) / this->r;
        double lb = (+ vx + vy + az * (a + b)) / this->r;
        double rb = (- vx + vy + az * (a + b)) / this->r;
        wheel_speed.insert(std::make_pair("right_front", rf));
        wheel_speed.insert(std::make_pair("left_front", lf));
        wheel_speed.insert(std::make_pair("left_back", lb));
        wheel_speed.insert(std::make_pair("right_back", rb));

    } else if (wheel_offline == WHEEL_OFFLINE_LF) {
        if (vx != 0 || vy != 0) {
            if (std::abs(vx) >= std::abs(vy)) {
                double lb = (+ vx) / this->r;
                double rb = (- vx) / this->r;
                wheel_speed.insert(std::make_pair("left_back", lb));
                wheel_speed.insert(std::make_pair("right_back", rb));
            } else {
                double rf = (- vy) / this->r;
                double rb = (+ vy) / this->r;
                wheel_speed.insert(std::make_pair("right_front", rf));
                wheel_speed.insert(std::make_pair("right_back", rb));
            }
        } else {
            double rf = (az * (a + b)) / this->r;
            double lb = (az * (a + b)) / this->r;
            wheel_speed.insert(std::make_pair("right_front", rf));
            wheel_speed.insert(std::make_pair("left_back", lb));
        }

    } else if (wheel_offline == WHEEL_OFFLINE_LB) {
        if (vx != 0 || vy != 0) {
            if (std::abs(vx) >= std::abs(vy)) {
                double rf = (- vx) / this->r;
                double lf = (+ vx) / this->r;
                wheel_speed.insert(std::make_pair("right_front", rf));
                wheel_speed.insert(std::make_pair("left_front", lf));
            } else {
                double rf = (- vy) / this->r;
                double rb = (+ vy) / this->r;
                wheel_speed.insert(std::make_pair("right_front", rf));
                wheel_speed.insert(std::make_pair("right_back", rb));
            }
        } else {
            double rf = (az * (a + b)) / this->r;
            double lf = (az * (a + b)) / this->r;
            wheel_speed.insert(std::make_pair("right_front", rf));
            wheel_speed.insert(std::make_pair("left_front", lf));
        }

    } else if (wheel_offline == WHEEL_OFFLINE_RF) {
        if (vx != 0 || vy != 0) {
            if (std::abs(vx) >= std::abs(vy)) {
                double lb = (+ vx) / this->r;
                double rb = (- vx) / this->r;
                wheel_speed.insert(std::make_pair("left_back", lb));
                wheel_speed.insert(std::make_pair("right_back", rb));
            } else {
                double lf = (- vy) / this->r;
                double lb = (+ vy) / this->r;
                wheel_speed.insert(std::make_pair("left_front", lf));
                wheel_speed.insert(std::make_pair("left_back", lb));
            }
        } else {
            double lf = (az * (a + b)) / this->r;
            double rb = (az * (a + b)) / this->r;
            wheel_speed.insert(std::make_pair("left_front", lf));
            wheel_speed.insert(std::make_pair("right_back", rb));
        }

    } else if (wheel_offline == WHEEL_OFFLINE_RB) {
        if (vx != 0 || vy != 0) {
            if (std::abs(vx) >= std::abs(vy)) {
                double rf = (- vx) / this->r;
                double lf = (+ vx) / this->r;
                wheel_speed.insert(std::make_pair("right_front", rf));
                wheel_speed.insert(std::make_pair("left_front", lf));
            } else {
                double lf = (- vy) / this->r;
                double lb = (+ vy) / this->r;
                wheel_speed.insert(std::make_pair("left_front", lf));
                wheel_speed.insert(std::make_pair("left_back", lb));
            }
        } else {
            double rf = (az * (a + b)) / this->r;
            double lb = (az * (a + b)) / this->r;
            wheel_speed.insert(std::make_pair("right_front", rf));
            wheel_speed.insert(std::make_pair("left_back", lb));
        }
    }

    return wheel_speed;
}