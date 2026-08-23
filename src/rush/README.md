# Rush

Rush 是一个导航架构包，目标是将感知、Topo 全局规划、局部路线规划、路径优化、速度规划与控制串联起来，完成普通道路和特殊地形上的连续导航。

## 通信

一切从简，导航希望通信局限在以下几个话题/服务

| 通信方 | 类型 | 话题名 | 话题类型 | 作用 |
| :--- |:--- |:---|:---|:---|
| decision | 服务 | /rush/nav_executor_ctx | robot_msgs/srv/NavExecutorCtx | 启动时确认双方使用同一套地图配置 |
| decision | 话题 | /rush/plan/local_plan | robot_msgs/msg/LocalPlan | 发送控制量规划结果 |
| protocol | 话题 | /protocol/recv_odom | nav_msgs/msg/Odometry | 接收控制量的反馈值 |
| protocol | 话题 | /rush/input/chassis_state_fdb | std_msgs/msg/Int8 | 接收底盘状态反馈值 |
| localization | 话题 | /Odometry/imu_interpolation | nav_msgs/msg/Odometry | 接收定位数据 |
| decision | 服务 | /rush/input/nav_command | robot_msgs/srv/NavCommand | 接收导航任务从而启动导航程序 |
| localization | 话题 | /cloud_registered | sensor_msgs/msg/PointCloud2 | 接收点云更新局部地图 |
| decision | 话题 | /rush/status/nav_event | robot_msgs/msg/NavEvent | 发送导航执行任务过程中的一系列事件，包括到达目标点，正常运行中等 |
| deicision | 话题 | /rush/status/nav_replan_status | robot_msgs/msg/NavReplanStatus | 发送重规划信息 | 

## 可视化

为更好的做调试，导航会自己发送一些debug信息

## 日志

## 项目结构

本项目分为`感知`和`运动`两大版块。

### 感知

本项目的感知分为`全局地图`和`局部地图`两个部分，全局地图通常由先验的点云建图而来
