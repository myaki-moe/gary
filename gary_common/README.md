# gary_common

gary_common是gary_ros2的通用部分, 通用节点 (自检消息聚合器, 生命周期节点管理器) 等

## DiagnosticAggregator

自检消息聚合器, 负责将多个节点发送的自检信息聚合到一个话题上, 并标记过期的自检信息

`DiagnosticAggregator`可配置参数如下

* diagnose_topic: 接收其他节点发送的自检数据主题名称, 默认/diagnostics
* agg_topic: 发送的聚合后主题名称, 默认/diagnostics_agg
* update_freq: 发送频率, 默认10Hz
* stale_threshold: 过期阈值, 默认0.5S

### Usage

直接启动节点
```bash
ros2 run gary_common diagnostic_aggregator
```

作为component加载
```bash
ros2 run rclcpp_components component_container
ros2 component load /ComponentManager gary_common gary_common::DiagnosticAggregator
```

注意, `DiagnosticAggregator`是生命周期节点, 当节点启动后, 需要配置才能进入运行状态
```bash
ros2 lifecycle set /diagnostic_aggregator configure
ros2 lifecycle set /diagnostic_aggregator activate
```

## LifecycleManager

生命周期节点管理器, 负责自动配置和激活生命周期节点, 并自动转换节点状态, 以及重新配置和激活失败的节点

`LifecycleManager`可配置参数如下

* node_names: 管理的生命周期节点名称, 默认为空
* diagnose_topic: 发送的自检信息主题名称, 默认/diagnostics
* diag_freq: 自检信息发送频率, 默认10Hz
* respawn: 在节点失败后, 是否重新配置节点, 默认True
* update_rate: 更新监控的节点状态频率, 默认10Hz

### Usage

直接启动节点
```bash
ros2 run gary_common lifecycle_mamager
```

作为component加载
```bash
ros2 run rclcpp_components component_container
ros2 component load /ComponentManager gary_common gary_common::LifecycleManager
```
