# gary_hardware

gary_hardware包基于ros2_control包提供的框架, 是gary_ros2中负责底层硬件读取和控制的封装, 实现了下层硬件资源的抽象. 包括can总线电机的通信, 以及can总线IMU的读取, 其他can总线设备 (CAN2GPIO, CAN2PWM等) 也能被轻松控制.

gary_hardware是实时就绪 (Realtime-Ready) 的, 经过优化的代码旨在提供以超过1KHz读写硬件的能力.

## hardware

### RMMotorSystem

RMMotorSystem是一种ros2_control中system类型的虚拟硬件, 向下能够读写一组相同can总线指令id的电机 (最多4个) , 向上能暴露每个电机的状态接口 (state interface) 和指令接口 (command interface)  
RMMotorSystem作为插件被加载到 ros2_control_node 中并被执行, 通过解析 URDF 获得电机的描述.

#### Usage

首先在URDF中描述硬件电机的数量和类型, 包括这一组电机使用的can总线名称, can_id, 以及每个电机的名称, id和类型.  
应当注意的是, 每一组cmd_id相同的电机只能由一个RMMotorSystem驱动, 多个RMMotorSystem可以驱动不同cmd_id的电机, 每个RMMotorSystem中的电机数量最多4个, 最少1个.
```xml
<robot name="rrbot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <ros2_control name="chassis_motors" type="system">
        <hardware>
            <plugin>gary_hardware/RMMotorSystem</plugin>
            <param name="can_bus">can3-7</param>
            <param name="cmd_id">0x200</param>
        </hardware>

        <joint name="chassis_left_front">
            <param name="motor_id">2</param>
            <param name="motor_type">m2006</param>
        </joint>

        <joint name="chassis_left_back">
            <param name="motor_id">3</param>
            <param name="motor_type">m3508</param>
        </joint>

        <joint name="chassis_right_front">
            <param name="motor_id">1</param>
            <param name="motor_type">m3508</param>
        </joint>

        <joint name="chassis_right_back">
            <param name="motor_id">4</param>
            <param name="motor_type">m3508</param>
        </joint>

    </ros2_control>
</robot>
```

在launch文件中通过参数向`ros2_control_node`传入URDF描述文件, `ros2_control_node`会自动加载对应的硬件.
```python
robot_description_content = Command(
    [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [
                FindPackageShare("gary_description"),
                "urdf",
                "demo.urdf",
            ]
        ),
    ]
)
robot_description = {"robot_description": robot_description_content}
control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description],
    )
```

节点启动完成后, 可以通过`ros2 control list_hardware_interfaces`展示所有interface, 确认硬件是否成功加载
```bash
command interfaces
	chassis_left_back/effort [unclaimed]
	chassis_left_front/effort [unclaimed]
	chassis_right_back/effort [unclaimed]
	chassis_right_front/effort [unclaimed]
state interfaces
	chassis_left_back/effort
	chassis_left_back/offline
	chassis_left_back/position
	chassis_left_back/temperature
	chassis_left_back/velocity
	chassis_left_front/effort
	chassis_left_front/offline
	chassis_left_front/position
	chassis_left_front/temperature
	chassis_left_front/velocity
	chassis_right_back/effort
	chassis_right_back/offline
	chassis_right_back/position
	chassis_right_back/temperature
	chassis_right_back/velocity
	chassis_right_front/effort
	chassis_right_front/offline
	chassis_right_front/position
	chassis_right_front/temperature
	chassis_right_front/velocity
```

### rm_imu_sensor

RMIMUSensor是一种ros2_control中sensor类型的虚拟硬件, 向下能够读取挂载在can总线上一个的IMU , 向上能暴露状态接口 (state interface)  
RMIMUSensor作为插件被加载到 ros2_control_node 中并被执行, 通过解析 URDF 获得IMU的描述.

#### Usage

首先在URDF中描述硬件IMU的can总线和can_id.  
应当注意的是, 每一个IMU都有3个can_id, 分别发送不同数据. 多个RMIMUSensor可以读取不同cmd_id的IMU.
```xml
<?xml version="1.0"?>
<robot name="rrbot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <ros2_control name="gimbal_imu" type="sensor">
        <hardware>
            <plugin>gary_hardware/RMIMUSensor</plugin>
            <param name="can_bus">can3-2</param>
            <param name="orientation_can_id">0x100</param>
            <param name="gyro_can_id">0x101</param>
            <param name="accel_can_id">0x102</param>
            <param name="update_rate">250</param>
        </hardware>
    </ros2_control>

</robot>
```

在launch文件中通过参数向`ros2_control_node`传入URDF描述文件, `ros2_control_node`会自动加载对应的硬件.
```python
robot_description_content = Command(
    [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [
                FindPackageShare("gary_description"),
                "urdf",
                "demo.urdf",
            ]
        ),
    ]
)
robot_description = {"robot_description": robot_description_content}
control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description],
    )
```

节点启动完成后, 可以通过`ros2 control list_hardware_interfaces`展示所有interface, 确认硬件是否成功加载
```bash
state interfaces
	gimbal_imu/angular_velocity.x
	gimbal_imu/angular_velocity.y
	gimbal_imu/angular_velocity.z
	gimbal_imu/linear_acceleration.x
	gimbal_imu/linear_acceleration.y
	gimbal_imu/linear_acceleration.z
	gimbal_imu/offline
	gimbal_imu/orientation.w
	gimbal_imu/orientation.x
	gimbal_imu/orientation.y
	gimbal_imu/orientation.z
```


## utils

### fp16_convert

fp16_convert提供了IEEE-754标准下全精度浮点数 (float32) 与半精度浮点数 (fp16) 之间的转换

#### Usage

```c++
#include "utils/fp16_convert.hpp"

float half_to_float(const ushort x);
ushort float_to_half(const float x);
```

### offline_detector

OfflineDetector提供了检测硬件掉线的功能, 通过手动触发更新, 其能够以设定的检测周期检测硬件是否离线.

若在一个检测周期之内没有更新或更新全部为失败, 则硬件被标记离线.

#### Usage

包含头文件
```c++
#include "utils/offline_detector.hpp"
```

为每一个需要检测离线的硬件实例化`OfflineDetector`, 构造函数需要传入检测周期
```c++
auto offlineDetector = std::make_shared<utils::OfflineDetector>(threshold);
```

与硬件交互后更新状态, 根据硬件读写是否成功, 成功传入true, 失败传入false. 注意, 一次读写失败不会导致硬件立刻被标记为离线, 而是在一个周期结束后仍没有成功读写的硬件会被标记为离线.
```c++
//read/write succ
offlineDetector->update(true);

//read/write failed
offlineDetector->update(false);
```

判断硬件是否在线
```c++
if(offlineDetector->offline) {
    //offline
}
```

### rm_motors

rm_motors封装了DJI Robomaster系列电机的通信协议, 能够对电机通信进行编解码和校验.

#### Usage

包含头文件
```c++
#include "utils/rm_motors.hpp"
```

为每个电机实例化一个`RMMotor`对象, 需要传入`MOTOR_TYPEDEF`定义的电机类型和电机id
```c++
auto motor = std::make_shared<utils::RMMotor>(utils::M3508, motor_id);
```

当需要生成电机控制指令时, 使用`cmd(effort_set)`设定力矩, 函数返回true成功, 返回false设定值超出范围.  
数组`motor->control_cmd[2]`存储的即是控制指令
```c++
struct can_frame frame{};
frame.can_dlc = 8;
frame.can_id = 0x200;

//set cmd
motor->cmd(1.0f);

//copy data
frame.data[0] = motor->control_cmd[0];
frame.data[1] = motor->control_cmd[1];
```

当需要解码电机反馈数据时, 使用`feedback(uint8_t fdb_data[8])`解码数据. 函数返回true成功, 返回false解码失败.
```c++
struct can_frame can_recv_frame{};
//read feedback data
// ......
//decode
if (motor->feedback(can_recv_frame.data)) {
    //decode success
} else {
    //decode failed
}
```

解码后数据存放在`motor->feedback_data[]`, 类型是`std::map<std::string, std::shared_ptr<double>>`, 可以读取电机能够的数据, 例如
```c++
motor->feedback_data["position"]
motor->feedback_data["velocity"]
motor->feedback_data["effort"]
motor->feedback_data["temperature"]
```
