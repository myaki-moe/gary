# gary_gimbal
gary_gimbal是负责云台控制的包，接收遥控器及视觉数据，实现云台yaw，pitch轴旋转。
## gimbal_teleop
`gimbal_teleop`节点负责获取遥控器及视觉数据，通过`gimbal_pitch_max`及`gimbal_pitch_min`实现Pitch轴的机械限位，其数值由人为校准得到。由于Yaw轴不需限位因此没有相应参数。

**autoaim_topic**

视觉发送的是否存在目标数据，当存在目标且遥控器右拨杆为`SW_UP`时开启自瞄模式。

**yaw_set_topic**

Yaw轴数据，为增量型。当视觉或遥控器数据传过来时会将其增加到该数据上。之后会传入`gimbal_control`节点中。

**yaw_set_topic**

Pitch轴数据，与Yaw轴数据一致。
## gimbal_control
`gimbal_control`节点负责接收`gimbal_teleop`节点传输的数据，并对其做一定处理，再将其发送至相应电机的PID节点上实现闭环。

**gimbal_yaw_ecd_transform**

Yaw轴电机编码器零点至车头位置编码器的值，由人为校准得到。通过读取上电时陀螺仪及编码器的数据（`motor_yaw_angle_pre`及`this->imu_yaw_angle_pre`），将其做差，得到当前位置与编码器原点的差值，并加上`gimbal_yaw_ecd_transform`，重新设置陀螺仪的零点。

**pitch_soft_limit**

Pitch轴软限位，在机械限位之上添加较小的偏移量，防止损坏机械部分。