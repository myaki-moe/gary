# gary_can
gary_can是负责can收发gary_ros包, 基于linux的socketcan, 实现了can收发和总线监控, 具有断线重连能力, 所有功能被封装为动态链接库供其他包调用.

gary_can包含三部分, 分别是can发送`SocketCANSender`, can接收`SocketCANReceiver` 和 can总线监控`SocketCANMonitor`

## Usage

在`package.xml`中引用gary_can
```xml
<depend>gary_can</depend>
```

在`CMakeLists.txt`中调用gary_can
```cmake
find_package(gary_can REQUIRED)
ament_target_dependencies(
        <YOUR_TARGET_NAME>
        ......
        gary_can
)
```

在代码中引用头文件
```c++
#include "gary_can/socket_can_receiver.hpp"
#include "gary_can/socket_can_sender.hpp"
```

## 模块介绍

### SocketCANSender

`SocketCANSender`是can发送类, 封装了can非阻塞发送相关功能. 每个SocketCANSender负责一路can总线的发送.

#### Usage

使用`ip link`显示的can接口名称实例化SocketCANSender
```c++
auto can_sender = std::make_shared<driver::can::SocketCANSender>("can*");
```

打开socket, 函数成功返回true, 失败返回false
```c++
can_sender->open_socket();
```
或重新指定can接口并打开socket, 函数成功返回true, 失败返回false
```c++
can_sender->open_socket("can*");
```

每次发送前都应检查`can_sender->is_opened`, 如果是false, 则应执行`can_sender->open_socket()`直到`can_sender->is_opened==true`

```c++
if (!can_sender->is_opened) {
    //reopen socket
    can_sender->open_socket();
}
```

使用`can_sender->send(frame)`向对应的can总线发送数据, 其中参数frame的类型是`struct can_frame`, 函数执行成功返回true, 失败返回false
```c++
struct can_frame frame{};
frame.can_dlc = 8;
//for example, 0x100
frame.can_id = 0x100;
frame.data[0] = 0x00;
// ......
frame.data[7] = 0x00;
if (can_sender->send(frame)) {
    //send succ
} else {
    //send failed
}
```


### SocketCANReceiver

`SocketCANReceiver`是can接收类, 封装了can非阻塞接收相关功能. 每个SocketCANReceiver负责一路can总线的接收.

#### Usage

使用`ip link`显示的can接口名称实例化SocketCANReceiver
```c++
auto can_receiver = std::make_shared<driver::can::SocketCANReceiver>("can*");
```

创建接收socket, 需要指定can总线id, 一个`SocketCANReceiver`实例可以打开多个不同can总线id的socket, 成功返回true, 失败返回false
```c++
can_receiver->open_socket(0x100);
can_receiver->open_socket(0x101);
......
can_receiver->open_socket(0x105);
can_receiver->open_socket(0x106);
```

每次接收前都应检查`can_receiver->is_opened[id]`, 如果是false, 则应执行`can_receiver->open_socket(id)`直到`can_receiver->is_opened[id]==true`

```c++
if (!can_receiver[id]->is_opened) {
    //reopen socket
    can_receiver->open_socket(id);
}
```

使用`can_receiver->read(id, &frame)`从对应的can总线接收对应id的数据, 其中参数frame的类型是`struct can_frame`, 函数执行成功返回true, 失败返回false
```c++
struct can_frame frame{};
if (can_receiver->read(id, &frame)) {
    //read succ
} else {
    //read failed
}
```

### SocketCANMonitor

`SocketCANMonitor`是can总线的监控程序, 能够检测总线的多种错误, 并发送自检数据到指定topic. 其能够检测设备离线, 传输干扰, 总线过载, 并回报负载率和每个id的发包频率.

#### Usage

直接启动节点
```bash
ros2 run gary_can socket_can_monitor
```

作为component加载
```bash
ros2 run rclcpp_components component_container
ros2 component load /ComponentManager gary_can gary_can::SocketCANMonitor
```

注意, `SocketCANMonitor`是生命周期节点, 当节点启动后, 需要配置才能进入运行状态
```bash
ros2 lifecycle set /socket_can_monitor configure
ros2 lifecycle set /socket_can_monitor activate
```

`SocketCANMonitor`可配置参数如下

* diagnose_topic: 发送的自检数据主题名称, 默认/diagnostics
* update_freq: 更新频率, 默认10Hz
* monitored_can_bus: 需要监控的can总线名称, 默认为空
* overload_threshold: can过载阈值, 默认为0.8