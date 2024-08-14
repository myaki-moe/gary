# gary_bringup
gary_bringup是gary_ros2的节点启动系统, 负责拉起并配置所有节点.

## gary_ros2.launch.py

`gary_ros2.launch.py`能够启动`rclcpp_components`节点容器, 并根据配置文件加载节点和参数, 以及启动`ros2_control_node`, 并根据配置文件加载`ros2_control`的控制器

配置文件共有三种类型, 分别为机器人描述文件urdf, ros2_control控制器参数, 常规component节点参数

### usage

假设机器人名称为<robot_name>

期望加载的控制器名称为<controller_name>

期望运行的component节点所在包名为<package_name>

期望运行的component节点名为<node_name>


**机器人硬件配置**

编写gary_description/urdf/ros2_control/<robot_name>.urdf, 此文件用来描述机器人硬件

**ros2_control控制器配置**

编写gary_bringup/config/<robot_name>/ros2_control/<xxx>.yaml

其中, 每个控制器都需要一个名为`controller_type`的参数表示其类型

**component节点配置**

编写gary_bringup/config/<robot_name>/<package_name>/<node_name>.yaml

应当注意的是, component节点的参数与ros2标准节点参数格式不同, 需要进行转换.

launch会按照<package_name>::<node_name>寻找component节点

**启动机器人**

```bash
ros2 launch gary_bringup gary_ros2.launch.py robot_type:=<robot_name>
```

### demo

**机器人硬件配置**

gary_description/urdf/ros2_control/2022_single_shooter_infantry.urdf

```xml
<?xml version="1.0"?>
<robot name="2022_single_shooter_infantry" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <ros2_control name="chassis_motors" type="system">
        <hardware>
            <plugin>gary_hardware/RMMotorSystem</plugin>
            <param name="can_bus">can-chassis</param>
            <param name="cmd_id">0x200</param>
        </hardware>

        <joint name="chassis_left_front">
            <param name="motor_id">2</param>
            <param name="motor_type">m3508</param>
        </joint>

    </ros2_control>
</robot>
```

**ros2_control控制器配置**

gary_bringup/config/2022_single_shooter_infantry/ros2_control/chassis_controllers.yaml

```yaml
chassis_lf_pid:
  ros__parameters:
    controller_type: gary_controller/PIDController
    command_interface: chassis_left_front/effort
    state_interface: chassis_left_front/velocity
    kp: 1.0
    ki: 0.003
    kd: 0.0
    max_out: 6.0
    max_iout: 0.75
```

**component节点配置**

gary_bringup/config/2022_single_shooter_infantry/gary_common/DiagnosticAggregator.yaml

```yaml
diag_freq: 10.0
diagnose_topic: /diagnostics
node_names:
- /diagnostic_aggregator
- /dr16_receiver
- /socket_can_monitor
- /mecanum_chassis_solver
- /chassis_teleop
respawn: false
update_rate: 10.0
```
