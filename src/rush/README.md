# Rush

Rush 是一个导航架构包，目标是将感知、Topo 全局规划、局部路线规划、路径优化、速度规划与控制串联起来，完成普通道路和特殊地形上的连续导航。

## 导航行为树

行为树以固定频率重复 Tick。感知与地图在每次规划前更新；Topo 规划维护全局通道；局部规划根据当前 Topo 边选择普通规划或带地形约束的规划。子树失败时先尝试局部恢复，持续失败才上升到 Topo 层换路。

下面使用 BehaviorTree.CPP 4 的 XML 结构表达整体运行逻辑：

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <!-- 每次 Tick 都优先检查紧急状态，否则继续导航。 -->
    <ReactiveFallback name="NavigationRoot">
      <Sequence name="EmergencyHandling">
        <IsEmergency/>
        <EmergencyStop/>
      </Sequence>

      <Sequence name="ContinuousNavigation">
        <SubTree ID="UpdatePerception"/>

        <!-- 已有路线失效时重新进行 Topo 搜索。 -->
        <Fallback name="EnsureTopoRoute">
          <IsTopoRouteValid/>
          <Sequence>
            <PlanTopoRoute/>
            <PublishNavigationIntent/>
          </Sequence>
        </Fallback>

        <!-- 根据当前 Topo 边改变地图解释方式。 -->
        <ApplyNavigationIntentToMap/>

        <!-- 规划失败时进入恢复子树。 -->
        <Fallback name="PlanOrRecover">
          <IfThenElse>
            <IsTerrainEdge/>
            <SubTree ID="TerrainPlanning"/>
            <SubTree ID="NormalPlanning"/>
          </IfThenElse>
          <SubTree ID="PlanningRecovery"/>
        </Fallback>
      </Sequence>
    </ReactiveFallback>
  </BehaviorTree>

  <BehaviorTree ID="UpdatePerception">
    <Sequence>
      <Parallel success_count="2" failure_count="1">
        <UpdateLocalizationAndVehicleState/>
        <UpdatePointCloudAndDynamicObstacles/>
      </Parallel>
      <UpdateGlobalLocalAndSemanticMap/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="NormalPlanning">
    <Sequence>
      <!-- 路线搜索负责避障，优化器不负责发现绕行通道。 -->
      <SearchKinematicCollisionFreePath/>
      <OptimizeLocalPath/>
      <PlanFeasibleSpeedProfile/>
      <CheckPathAndSpeedFeasibility/>
      <FollowTrajectorySegment/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="TerrainPlanning">
    <Sequence>
      <!-- 只屏蔽当前地形本体，保留真实静态和动态障碍物。 -->
      <ActivateTerrainMapPolicy/>
      <SearchKinematicCollisionFreePath/>
      <ExtendHorizonThroughTerrainExit/>

      <!-- 路径层保证朝向、曲率和通行区域约束。 -->
      <OptimizeTerrainPath/>

      <!-- 速度层保证加减速、横向加速度和最低通过速度。 -->
      <PlanTerrainSpeedProfile/>
      <CheckPathAndSpeedFeasibility/>
      <FollowTrajectorySegment/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="PlanningRecovery">
    <Fallback>
      <!-- 根据失败原因换路线、增加准备距离或调整优化窗口。 -->
      <RetryLocalPlanning/>

      <Sequence>
        <IsTemporarilyBlocked/>
        <SlowDownOrWait/>
      </Sequence>

      <!-- 持续不可行时使 Topo 路线失效，下一次 Tick 重新选路。 -->
      <InvalidateCurrentTopoEdge/>
    </Fallback>
  </BehaviorTree>
</root>
```

这里使用 `IfThenElse` 选择地形或普通规划，避免地形规划失败后错误地退回普通规划。地形规划失败会进入 `PlanningRecovery`：局部可恢复的问题继续重规划，动态阻挡可以等待，持续不可行才禁用当前 Topo 边。

整体采用分层规划：Topo 全局规划只决定经过哪些节点、边和语义区域，不规划完整的精细路径；局部路线规划负责在实时障碍物中寻找可通行路线；路径优化负责平滑路线并满足几何约束；速度规划负责验证和生成动力学可行的速度曲线；控制器只负责跟踪和小范围误差修正。

## 感知与地图

### 全局地图

全局地图由先验地图提供，使用其中的梯度或占据信息判断静态障碍物，并可选择是否进行障碍物膨胀。它主要服务于 Topo 全局规划和局部规划的全局引导。

### 局部地图

局部地图计划采用 `ROG-Map` 处理实时 3D 点云，生成局部 2D 占据地图和 ESDF 场，为局部路线搜索、路径优化和动态避障提供信息。

### 地形与真实障碍物

特殊地形需要与普通障碍物分层表达。先验地图记录地形区域、地形 ID 和通行约束；实时感知继续检测地形附近的车辆、人员和其他障碍物。

当 Topo 规划选择某条地形边时，地图模块只屏蔽该地形本体产生的占据信息，不能直接清空整个地形区域，否则会同时删除地形上的真实障碍物。早期可以依据先验地形表面和高度差过滤地形点，后续再加入点云分割和动态目标跟踪。

### 语义拓扑地图

语义地图对先验地图进行区域划分，使其能够表达 `高地`、`台阶 1`、`基地` 等任务语义。决策端只需要给出区域通过顺序。

### 几何拓扑地图

几何 Topo 地图由节点和边构成，用于表达道路、岔路和特殊地形之间的连通关系。特殊地形边除连通关系外，还需要携带：

- 地形区域、准备区域和出口区域；
- 推荐通行方向及最大朝向误差；
- 最低和最高通过速度；
- 允许的最大曲率等运动约束；
- 激活该边时对应的地图解释策略。

## 规划

### 语义规划

根据比赛规则将全局先验地图划分为多个区域，决策端负责指定区域通过顺序，例如 `高地 -> 台阶 1 -> 基地`。

### Topo 全局规划

Topo 全局规划根据机器人位置、目标区域和语义通过顺序选择拓扑路径。它输出的不是完整轨迹，而是当前 Topo 边、后续通道、目标出口区域和对应约束。

Topo 与障碍物地图双向联动：Topo 边决定当前允许将哪一个地形解释为有条件可通行区域；局部规划发现入口被阻挡或约束不可满足时，则提高该边代价或暂时禁用该边并触发 Topo 重规划。

### 局部路线规划

局部路线规划负责避障，不将绕障决策交给速度规划器或控制器。规划应在包含位置和朝向的状态空间中进行，并使用符合车辆运动学的扩展方式，使输出路线基本满足最大曲率、车辆轮廓和地形方向约束。

局部路线不要求沿固定的地形跨越线行驶，只要能够在允许区域内绕开真实障碍物，并在地形核心区域满足朝向、曲率和通行方向约束即可。

### 路径优化与平滑

搜索得到的路线需要进行平滑和几何优化。普通道路只优化未来一段距离；前方存在特殊地形时，动态扩展优化窗口，使它完整覆盖：

```text
避障与调整段 -> 对准和加速段 -> 地形核心段 -> 稳定驶离段
```

这样既避免优化整条全局路径带来的计算浪费，也防止局部窗口过短，直到地形入口前才发现无法对准或加速。

### 速度规划

速度规划沿优化后的路径生成速度曲线，处理以下约束：

- 车辆最大和最小速度；
- 纵向加速度和减速度；
- 路径曲率对应的横向加速度上限；
- 地形核心区域内的最低通过速度；
- 入口前的可用加速距离以及出口后的减速距离。

速度规划不负责空间避障，但需要验证路线是否具有动力学可行的速度曲线。如果地形最低速度与路径曲率允许的最高速度冲突，或者入口前没有足够的加速距离，应将明确的失败原因反馈给路线规划和路径优化，而不是强行生成速度。

规划和控制频率由实际计算开销与底盘响应共同确定，速度与控制环期望保持在约 `40 Hz`。

## 特殊地形跨越

地形跨越是一段连续运动，不要求车辆停车、切换到固定路线后再启动。Topo 提前给出地形区域和约束，局部规划从准备区开始连续调整路线与速度，并根据实时状态滚动修正。

地形约束在准备区域内逐渐增强，在核心区域成为硬约束，在驶离后逐渐解除。局部规划器每次只执行轨迹前段，然后基于最新位姿、速度和地图重新搜索与优化，从而在保持连续运动的同时修正实际误差。

动力学和地形约束通过分层验证保证：

1. 路线搜索保证无碰撞，并生成车辆运动学上基本可行的路线；
2. 路径优化保证地形处的朝向、曲率和通行区域约束；
3. 速度规划保证加减速、横向加速度和地形最低速度约束；
4. 路线与速度联合检查失败时，携带失败原因返回上一层重新规划；
5. 控制器跟踪最终路线和速度，并在误差超限时触发滚动重规划。

## 特殊情况处理

### 地形附近阻挡较多

路线规划器可以在地形允许区域内选择不同的接近路线，但每条候选路线都必须经过路径和速度联合检查。绕障后如果无法恢复通行方向或没有足够加速距离，即使当前路线无碰撞，也应判定为不可行。

### 动力响应缓慢

根据当前速度、可用加速度、路径曲率和地形最低速度动态计算准备距离。车辆动力越慢，越早扩展优化窗口并开始调整速度。若物理上不存在足够的加速距离，则更换接近路线或 Topo 通道。

### 动态障碍物进入地形

地形地图策略不能删除动态障碍物。尚未进入地形时可以减速、等待或换路；进入后仍持续感知和滚动规划。出现无法绕开的紧急危险时，安全策略的优先级高于完成跨越。

### 路线或速度不可行

规划失败应区分碰撞、朝向超限、曲率超限、加速距离不足和最低速度冲突。局部问题反馈给路线搜索和路径优化；当前地形通道持续不可行时，再反馈 Topo 层重新选择通道，避免无意义地重复相同规划。

### 重规划策略

地图变化、路线阻挡、跟踪偏差和约束不可满足都会触发局部重规划。只有当前 Topo 边持续不可行时才触发全局重规划，并通过最小保持时间、失败冷却时间和切换收益阈值避免在多条 Topo 边之间频繁跳转。
