# rviz_sim

`rviz_sim` 是一个基于分层栅格地图的二维机器人仿真器。它从 `ControlPlan` 接收平面速度，在地图上模拟移动、碰撞和 2D 雷达，并发布当前楼层地图、里程计、雷达数据和 TF。`sim.launch.py` 会同时启动仿真节点和 RViz2。

## 构建与启动

在工作区根目录执行：

```bash
./compile.sh
source install/setup.bash
ros2 launch rviz_sim sim.launch.py
```

启动文件使用包安装目录中的 `config/sim.yaml`、场景文件和 `rviz/sim.rviz`。仿真节点的工作目录设为 `rviz_sim` 的 package share，因此默认的相对场景路径可以直接使用。关闭 launch 会同时退出仿真器和 RViz2。

## 发送移动命令

仿真器订阅 `/gobot/control_plan`，消息类型为 `robot_msgs/msg/ControlPlan`：

```text
std_msgs/Header header
float32 vx
float32 vy
```

`vx`、`vy` 是沿 map 坐标系 x、y 方向的速度，单位为 m/s。当前实现只读取这两个速度字段；`header` 不参与控制或超时计算。可以用 ROS 2 CLI 连续发送一个低速命令：

```bash
ros2 topic pub -r 10 /gobot/control_plan robot_msgs/msg/ControlPlan "{vx: 0.2, vy: 0.0}"
```

按 Ctrl+C 停止发送。命令超过 `command_timeout` 未更新时，仿真器会把目标速度清零；默认超时为 0.5 秒。发生碰撞时机器人停止，并清除当前速度命令。

## 输入与输出

| 方向 | 话题 / TF | 类型或格式 | 行为 |
| --- | --- | --- | --- |
| 输入 | `/gobot/control_plan` | `robot_msgs/msg/ControlPlan` | 接收 `vx`、`vy`；订阅队列深度为 1。 |
| 输出 | `/gobot/map` | `nav_msgs/msg/OccupancyGrid` | 发布当前楼层整张地图；启动加载地图时发布，楼层变化时重新发布。使用 Reliable、Transient Local QoS，晚加入的订阅者也能收到最近一次地图。 |
| 输出 | `/gobot/scan` | `sensor_msgs/msg/LaserScan` | 每个仿真 tick 发布一圈 360 束扫描，范围 0.05–10 m；使用 Reliable QoS、Keep Last 深度 1。扫描在单个仿真时刻计算，`time_increment` 为 0。 |
| 输出 | `/gobot/odom` | `nav_msgs/msg/Odometry` | 发布机器人在 `map` 下的位置和当前速度。 |
| 输出 | `/tf` | `geometry_msgs/msg/TransformStamped` | 发布 `map → base_link` 动态变换。 |

仿真 tick 周期为 50 ms（20 Hz）。地图、scan、odom 和 TF 的消息时间戳使用仿真节点时钟；当前 launch 默认使用系统时间。固定坐标系是 `map`，机器人坐标系是 `base_link`，TF frame 名称不带 `gobot` 前缀。

当前只发布完整的 `/gobot/map`，没有地图增量话题 `/gobot/map_updates`。地图只在初始化和楼层切换时发布，不会按 tick 重发。

## 参数

参数位于 `config/sim.yaml` 的 `/gobot/simulator.ros__parameters` 下：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `initial_x` | `0.0` | 初始 map x 坐标，单位 m。 |
| `initial_y` | `0.0` | 初始 map y 坐标，单位 m。 |
| `initial_floor` | `1` | 初始化所在楼层；对应楼层必须存在于场景配置中。楼层在启动时确定，当前没有运行时切层接口。 |
| `robot_radius` | `0.25` | 圆形机器人半径，单位 m；碰撞检查使用此值。 |
| `command_timeout` | `0.5` | 收到速度命令后的有效时间，单位 s；超时后目标速度归零。 |
| `scene_config` | `scene/scene_example/scene.yaml` | 分层场景配置文件路径。通过 launch 启动时，相对路径以 `rviz_sim` 的 package share 为基准。 |

修改参数后重新启动仿真即可。launch 文件目前没有暴露自定义参数文件的 launch 参数；如需换场景，编辑 `sim.yaml` 中的 `scene_config`，并确保场景文件及地图图片安装在包的 `scene/` 目录下，然后重新构建并启动。

## 场景与地图

一个场景配置描述多个楼层，每层引用 Nav2 map server 可读取的地图 YAML。示例目录如下：

```text
scene/scene_example/
├── scene.yaml
├── floor1/
│   ├── map.yaml
│   └── map.png
└── floor2/
    ├── map.yaml
    └── map.png
```

`scene.yaml` 示例：

```yaml
floors:
  - floor: 1
    map: floor1/map.yaml
  - floor: 2
    map: floor2/map.yaml
```

`map` 路径相对于 `scene.yaml` 所在目录；地图 YAML 中的 `image` 路径相对于该地图 YAML。每张地图的分辨率、原点和占用栅格由地图 YAML 与图片提供。启动时加载所有楼层地图，`initial_floor` 选择初始地图。

碰撞与雷达都基于当前楼层的 `OccupancyGrid`。占用值小于 0（未知）或大于等于 50 的格子视为障碍；地图范围以外也视为不可通行。碰撞模型按 `robot_radius` 检查机器人圆形范围，雷达沿 360 个方向查找最近障碍物。

## 仿真行为与限制

- 机器人每 50 ms 更新一次位置。收到的速度命令会叠加约为命令速度 2% 标准差的高斯扰动，以模拟执行误差。
- 碰撞时机器人该 tick 不移动，并清除当前速度命令；地图外和未知区域会阻挡机器人。
- scan 在当前机器人位姿下瞬时计算，`scan_time` 表示 50 ms 的扫描周期，`time_increment` 为 0。这样 RViz2 不会把同一帧中的后续光束解释成未来时刻。
- 电梯目前没有实现：`Robot::take_elevator` 只有声明，没有定义；仿真节点也没有提供电梯 topic、service 或 action。初始楼层只能通过启动参数设置。
- 仿真环境是二维栅格地图，不包含楼层间的 3D 几何、电梯运动过程或完整物理动力学。

## 在此基础上扩展

- **增加楼层或场景：**在场景目录增加地图 YAML 和图片，并在 `scene.yaml` 的 `floors` 列表登记楼层；确认 `initial_floor` 指向有效楼层。
- **扩展地图管理：**`LayeredMap` 负责加载场景、按楼层读取地图和更新地图。需要地图编辑或运行时更新时，优先在该类扩展，再由 `Simulator` 发布当前楼层地图。
- **扩展运动或碰撞：**在 `Robot` 中放置机器人自身状态和运动行为，在 `Simulator::tick()` 中处理命令、地图碰撞和周期更新。保持速度命令超时与 map 边界碰撞行为明确。
- **增加传感器输出：**在 `Simulator` 中创建以 `/gobot/` 开头的 publisher，并在 tick 中生成消息。消息时间戳应与生成该数据时的机器人状态一致，`header.frame_id` 应使用 TF 树中的 frame；若传感器按一帧内的采样时间逐束生成，再设置非零 `time_increment`。
- **接入电梯：**为电梯操作定义明确的 ROS 接口并实现楼层切换流程；切层时需要同步机器人楼层、当前地图和 `/gobot/map` 发布。不要直接改写当前楼层状态绕过该流程。

头文件位于 `include/rviz_sim/`。`Simulator` 是 ROS 2 节点及话题/周期更新的入口，`Robot` 管理机器人运动和雷达检测，`LayeredMap` 独立管理分层地图。
