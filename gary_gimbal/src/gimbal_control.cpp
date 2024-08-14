//
// Created by maackia on 23-2-7.
//
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int16.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "gary_msgs/msg/dual_loop_pi_dwith_filter.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "gary_gimbal/gimbal_control.hpp"

using namespace std::chrono_literals;

class GimbalTask : public rclcpp::Node
{
public:
    GimbalTask():Node("gimbal_control"){
        this->declare_parameter("gimbal_pitch_max_ecd",0.739379);//TODO 写入至配置文件
        this->declare_parameter("gimbal_pitch_min_ecd",-0.461077);
        this->declare_parameter("gimbal_yaw_ecd_transform",1.66);
        this->declare_parameter("gimbal_pitch_ecd_transform",1.717291);
        init();
        yaw_pid_sub_ = this->create_subscription<gary_msgs::msg::DualLoopPIDwithFilter>("/gimbal_yaw_pid/pid",rclcpp::SystemDefaultsQoS(),std::bind(&GimbalTask::yaw_pid_callback,this, std::placeholders::_1));
        pitch_pid_sub_ = this->create_subscription<gary_msgs::msg::DualLoopPIDwithFilter>("/gimbal_pitch_pid/pid",rclcpp::SystemDefaultsQoS(),std::bind(&GimbalTask::pitch_pid_callback,this, std::placeholders::_1));
        joint_subscription = this->create_subscription<control_msgs::msg::DynamicJointState>("/dynamic_joint_states", rclcpp::SystemDefaultsQoS(), std::bind(&GimbalTask::joint_callback,this, std::placeholders::_1));
        yaw_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/gimbal_yaw_pid/set",rclcpp::SystemDefaultsQoS());
        pitch_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/gimbal_pitch_pid/set",rclcpp::SystemDefaultsQoS());
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/gimbal_imu_broadcaster/imu",rclcpp::SystemDefaultsQoS(),std::bind(&GimbalTask::imu_callback,this,std::placeholders::_1));
        state_sub_ = this->create_subscription<std_msgs::msg::Int16>("/gimbal_enter",rclcpp::SystemDefaultsQoS(),std::bind(&GimbalTask::state_callback,this,std::placeholders::_1));//TODO 信息类型待定
    }
private:

    void init(){
        gimbal::pitch.absolute_angle_max = this->get_parameter("gimbal_pitch_max_ecd").as_double();
        gimbal::pitch.absolute_angle_min = this->get_parameter("gimbal_pitch_min_ecd").as_double();
        gimbal::yaw.ecd_transform = this->get_parameter("gimbal_yaw_ecd_transform").as_double();
        gimbal::pitch.ecd_transform = this->get_parameter("gimbal_pitch_ecd_transform").as_double();
    }

    void yaw_pid_callback(const gary_msgs::msg::DualLoopPIDwithFilter::SharedPtr msg){
        gimbal::yaw_pid = *msg;
    }
    void pitch_pid_callback(const gary_msgs::msg::DualLoopPIDwithFilter::SharedPtr msg){
        gimbal::pitch_pid = *msg;
    }
    //四元数转换 ROLL,YAW反了
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
        gimbal::Imu = *msg;
        double yaw,pitch,roll;
        tf2::Quaternion imu_quaternion(gimbal::Imu.orientation.x,gimbal::Imu.orientation.y,gimbal::Imu.orientation.z,gimbal::Imu.orientation.w);
        tf2::Matrix3x3 m(imu_quaternion);
        m.getRPY(roll,pitch,yaw);
        //RCLCPP_INFO(this->get_logger(),"yaw:%lf pitch:%lf roll:%lf",yaw,pitch,roll);
        gimbal::yaw.absolute_angle = roll;
        gimbal::pitch.absolute_angle = pitch;
    }
    void joint_callback(control_msgs::msg::DynamicJointState::SharedPtr joint_state) {
        for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
            if (joint_state->joint_names[i] == "gimbal_yaw") {
                for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j) {
                    if (joint_state->interface_values[i].interface_names[j] == "encoder") {
                        double origin = joint_state->interface_values[i].values[j];
                        double fixed = origin;
                        fixed = origin - 1.66;
                        if (fixed < 0) fixed+=6.28;
                        if(fixed > PI) fixed-=2*PI;//转换为-PI到PI
                        gimbal::yaw.relative_angle = origin;
                        //RCLCPP_INFO(this->get_logger(), "origin %f fixed %f", origin, fixed);
                    }
                }
            }
        }
        for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
            if (joint_state->joint_names[i] == "gimbal_pitch") {
                for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j) {
                    if (joint_state->interface_values[i].interface_names[j] == "encoder") {
                        double origin = joint_state->interface_values[i].values[j];
                        double fixed = origin;
                        fixed = origin - 1.717291;
                        if (fixed < 0) fixed+=6.28;
                        if(fixed > PI) fixed-=2*PI;//转换为-PI到PI
                        gimbal::pitch.relative_angle = origin;
                        //RCLCPP_INFO(this->get_logger(), "origin %f fixed %f", origin, fixed);
                    }
                }
            }
        }
    }
    void state_callback(const std_msgs::msg::Int16::SharedPtr msg){
        gimbal::state = *msg;
        if (gimbal::mode.behaviour == GIMBAL_ABSOLUTE_ANGLE){//TODO 限位
            std_msgs::msg::Float64 pid;
            if (gimbal::yaw.absolute_angle_set <= PI && gimbal::yaw.absolute_angle_set >= -PI){
                gimbal::yaw.ecd_set = gimbal::yaw.absolute_angle_set + gimbal::yaw.ecd_transform;
                gimbal::yaw.ecd_delta = gimbal::yaw.ecd_set - gimbal::yaw.relative_angle;
                gimbal::yaw.pid_set = gimbal::yaw.ecd_delta - gimbal::yaw_pid.outer_feedback;//TODO check

                pid.data = gimbal::yaw.pid_set;
                yaw_publisher_->publish(pid);
            }
            if (gimbal::pitch.absolute_angle_set <= gimbal::pitch.absolute_angle_max && gimbal::pitch.absolute_angle_set >= gimbal::pitch.absolute_angle_min){
                gimbal::pitch.ecd_set = gimbal::pitch.absolute_angle_set + gimbal::pitch.ecd_transform;
                gimbal::pitch.ecd_delta = gimbal::pitch.ecd_set - gimbal::pitch.relative_angle;
                gimbal::pitch.pid_set = gimbal::pitch.ecd_delta - gimbal::pitch_pid.outer_feedback;//TODO check

                pid.data = gimbal::pitch.pid_set;
                pitch_publisher_->publish(pid);
            }
        }
    }
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_publisher_;
    rclcpp::Subscription<gary_msgs::msg::DualLoopPIDwithFilter>::SharedPtr yaw_pid_sub_;
    rclcpp::Subscription<gary_msgs::msg::DualLoopPIDwithFilter>::SharedPtr pitch_pid_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_subscription;
};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalTask>());
    return 0;
}