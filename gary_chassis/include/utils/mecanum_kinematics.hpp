#pragma once

#include <map>

namespace gary_chassis{

    enum wheel_offline_e{
        WHEEL_OFFLINE_NONE,
        WHEEL_OFFLINE_LF,
        WHEEL_OFFLINE_LB,
        WHEEL_OFFLINE_RF,
        WHEEL_OFFLINE_RB,
    };

    class MecanumKinematics {
    public:
        MecanumKinematics(double a, double b, double r);

        std::map<std::string, double> forward_solve(const std::map<std::string, double>& wheel_speed,
                                                    wheel_offline_e wheel_offline = WHEEL_OFFLINE_NONE) const;
        std::map<std::string, double> inverse_solve(const std::map<std::string, double>& chassis_speed,
                                                    wheel_offline_e wheel_offline = WHEEL_OFFLINE_NONE) const;

    private:
        double a;
        double b;
        double r;

    };
}