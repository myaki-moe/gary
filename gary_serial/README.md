# gary_serial

gary_serial是gary_ros2元包的一部分, 负责串口收发及数据编码解码, 包括DBUS接收, 串口收发, 接收机解码, 裁判系统编解码等.

## DR16Receiver

DR16接收机接收和解码, 负责打开串口, 解码数据包并将数据转发到topic, 其拥有断线重连和发送自检信息的能力, 能够检测串口硬件离线, 接收机离线, 传输干扰等情况.

`DR16Receiver`发送的数据范围如下:  
摇杆: -1.0f 到 1.0f  
拨轮: -1.0f 到 1.0f  
拨杆: 根据msg宏定义  
鼠标和键盘按键: bool  
鼠标移动: 原始数据, -32768.0f 到 32767.0f  

`DR16Receiver`可配置参数如下

* send_topic: 遥控数据发送主题, 默认/remote_control
* diagnostic_topic: 自检信息发送主题, 默认/diagnostics
* update_freq: 串口数据读取频率, 默认100Hz
* diag_freq: 自检信息发送频率, 默认10Hz
* serial_port: 串口设备名, 默认/dev/ttyDBUS0
* baudrate: 串口波特率, 默认100000
* override_diag_device_name: 覆盖默认的自检消息中的硬件名称, 默认为空


### Usage

直接启动节点
```bash
ros2 run gary_common dr16_receiver
```

作为component加载
```bash
ros2 run rclcpp_components component_container
ros2 component load /ComponentManager gary_serial gary_serial::DR16Receiver
```

注意, `DR16Receiver`是生命周期节点, 当节点启动后, 需要配置才能进入运行状态
```bash
ros2 lifecycle set /dr16_receiver configure
ros2 lifecycle set /dr16_receiver activate
```
