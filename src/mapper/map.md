# Occupancy2D 原理

Occupancy2D 的核心是将二维空间离散成栅格，并把每个栅格看成一个“是否被占用”的二值随机变量。

## 1. 栅格地图模型

对于第 $i$ 个栅格：

$$
m_i \in \{0,1\}
$$

- $m_i=1$：栅格被障碍物占用。
- $m_i=0$：栅格空闲。

整个地图为：

$$
M=\{m_1,m_2,\dots,m_N\}
$$

通常假设各个栅格相互独立：

$$
P(M \mid Z_{1:t},X_{1:t})
=
\prod_i P(m_i \mid Z_{1:t},X_{1:t})
$$

其中：

- $Z_{1:t}$：截至当前的激光雷达观测。
- $X_{1:t}$：机器人位姿。
- $P(m_i \mid Z_{1:t},X_{1:t})$：第 $i$ 个栅格被占用的概率。

这个独立性假设并不完全符合现实，但可以极大简化计算。

## 2. 世界坐标转换成栅格坐标

假设地图原点为 $(x_o,y_o)$，栅格分辨率为 $r$，那么世界坐标 $(x,y)$ 对应的栅格坐标为：

$$
g_x=\left\lfloor\frac{x-x_o}{r}\right\rfloor
$$

$$
g_y=\left\lfloor\frac{y-y_o}{r}\right\rfloor
$$

对应实现位于 `src/map.cpp` 的 `Map::world_to_grid()`。

必须使用 `floor`，不能直接转换成整数，否则负数坐标会产生错误：

```cpp
static_cast<int>(-0.2F); // 结果是 0
std::floor(-0.2F);       // 结果是 -1
```

## 3. 激光雷达射线模型

假设雷达在地图坐标系中的位姿为 $(x_s,y_s,\theta_s)$。第 $k$ 条激光束的局部角度为：

$$
\alpha_k=\alpha_{\min}+k\Delta\alpha
$$

它在地图坐标系中的角度为：

$$
\theta_k=\theta_s+\alpha_k
$$

如果测量距离为 $r_k$，激光终点为：

$$
x_e=x_s+r_k\cos(\theta_k)
$$

$$
y_e=y_s+r_k\sin(\theta_k)
$$

这部分由 `src/occupancy_map.cpp` 的 `OccupancyMap::scan_callback()` 完成。

随后使用 Bresenham 算法找出起点到终点经过的栅格：

```text
雷达  空闲  空闲  空闲  障碍物
  S -----> -----> -----> X
```

更新规则：

- 射线经过的栅格标记为空闲。
- 射线实际命中的终点标记为占用。
- 如果测量值达到最大量程，整条射线标记为空闲，不标记占用终点。

射线更新由 `src/map.cpp` 的 `Map::integrate_ray()` 完成。

## 4. 为什么不能直接设置成 0 或 100

激光雷达存在测量噪声，机器人位姿也可能不准确。如果一次命中就直接写入：

```cpp
grid[cell] = 100;
```

下一次没有命中又写入：

```cpp
grid[cell] = 0;
```

地图就会在空闲和占用之间不断跳变。因此需要累计多次测量提供的证据。

## 5. 贝叶斯递推

目标是持续更新：

$$
P(m_i \mid z_{1:t},x_{1:t})
$$

根据贝叶斯公式：

$$
P(m_i \mid z_{1:t})
=
\frac
{P(z_t \mid m_i)P(m_i \mid z_{1:t-1})}
{P(z_t \mid z_{1:t-1})}
$$

直接使用概率计算时需要不断执行乘法和归一化，因此 Occupancy Grid 通常使用 Odds：

$$
O(m_i)=\frac{P(m_i)}{1-P(m_i)}
$$

再取对数得到 Log-Odds：

$$
L(m_i)=\log\frac{P(m_i)}{1-P(m_i)}
$$

贝叶斯递推可以转化为加法：

$$
L_{t,i}
=
L_{t-1,i}
+
\log\frac{P(m_i \mid z_t)}{1-P(m_i \mid z_t)}
-
L_{0,i}
$$

如果先验占用概率为：

$$
P(m_i)=0.5
$$

那么：

$$
L_{0,i}=\log\frac{0.5}{0.5}=0
$$

递推公式可简化为：

$$
L_{t,i}=L_{t-1,i}+\Delta L_i
$$

## 6. 当前实现中的更新参数

当前采用：

```cpp
free_update = -0.4F;
occupied_update = 0.85F;
```

命中障碍物时：

$$
L_t=L_{t-1}+0.85
$$

射线经过空闲区域时：

$$
L_t=L_{t-1}-0.4
$$

单次障碍物观测对应的概率为：

$$
P=\frac{1}{1+e^{-0.85}}\approx0.70
$$

连续观测三次后：

$$
L=3\times0.85=2.55
$$

$$
P=\frac{1}{1+e^{-2.55}}\approx0.928
$$

因此一个栅格需要被重复观测，才会逐渐成为高可信度的障碍物。

## 7. 从 Log-Odds 恢复概率

发布地图前，通过 Sigmoid 恢复占用概率：

$$
P(m_i)=\frac{1}{1+e^{-L_i}}
$$

然后转换成 ROS `OccupancyGrid` 的整数值：

$$
value=\operatorname{round}(100P(m_i))
$$

对应实现位于 `src/map.cpp` 的 `Map::occupancy_data()`。

当前 Log-Odds 被限制在：

$$
-4\le L_i\le4
$$

对应的概率范围大约为：

$$
0.018\le P(m_i)\le0.982
$$

限制范围可以防止一个栅格积累大量历史观测后，完全无法被新观测纠正。

## 8. 未知栅格为什么需要单独保存

当 Log-Odds 为 $0$ 时：

$$
P=0.5
$$

但它可能表示两种情况：

1. 栅格从未被观测。
2. 空闲证据和占用证据恰好抵消。

因此实现中还维护了：

```cpp
std::vector<bool> observed_;
```

- 没有观测过的栅格发布为 `-1`。
- 已经观测过的栅格发布为 `0~100`。

这对自主探索非常重要，因为 Frontier 是已知空闲区域和未知区域之间的边界。

## 9. 当前模型的限制

当前方法假设机器人位姿准确，并使用以下 TF 链：

```text
odom -> base_footprint -> lidar_link
```

如果里程计产生漂移，同一个墙壁会被绘制到多个位置，地图会逐渐出现重影。仿真中的短距离测试问题不大，但长期自主探索需要加入 SLAM：

```text
SLAM
  -> 估计 map -> odom
  -> 修正机器人累计位姿误差
```

因此，当前实现完成了占据栅格感知模型，但还不是完整 SLAM。后续 Frontier Exploration 可以使用这张地图检测未知边界；长期运行前还需要补充位姿校正。
