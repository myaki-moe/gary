# gary_controller
gary_controller定义了ros2_control中的上层controller, 是gary_ros2中上层控制器的封装, 实现了多种控制算法的控制器, 拥有参数实时更改能力

gary_controller是实时就绪 (Realtime-Ready) 的, 经过优化的代码旨在提供以超过1KHz闭环控制的能力.


## usage

启动`ros2_control_node`  
```bash
ros2 run controller_manager ros2_control_node 
```

将控制器加载到`ros2_control_node`中并运行
```bash
ros2 run controller_manager spawner.py <controller_name> --controller-manager /controller_manager --param-file <param-file> --controller-type <controller_type>
```


## controller

### OfflineBroadcaster

硬件离线状态广播器, 负责读取所有硬件的offline状态接口, 并将状态广播到指定的topic上

`OfflineBroadcaster`可配置参数如下

* interface_name: 硬件暴露的掉线状态接口名称, 默认offline
* diagnose_topic: 发送的自检信息主题名称, 默认/diagnostics
* pub_rate: 发送频率, 默认10Hz


### PIDController

基本pid控制器, 其订阅`~/cmd`主题获得pid设定值, 与从state_interface拿到的反馈数据进行pid闭环运算, 将输出写入到command_interface

`PIDController`可配置参数如下

* command_interface: pid运算结果输出的指令接口, 默认为空
* state_interface: pid反馈输入的状态接口, 默认为空
* kp: 比例项, 默认为0
* ki: 积分项, 默认为0
* kd: 微分项, 默认为0
* max_out: pid总体最大输出, 默认为0
* max_iout: 积分项最大输出, 默认为0
* stale_threshold: pid设定值topic接收超时阈值, 超时将导致pid输出关闭, 默认为0.1(s)
