---
name: rush-code-design 
description: A tool to make the code in the rush package more readable and clear. Everyone who wants to modify the rush package should read this skill.
---

## Follow rules

1. 在不同线程间通信时，只通过读取/写入同一处的数据来沟通，并且这处数据要专门做加锁或者使用原子操作保证线程安全
2. 严格遵守各个模块的职责划分，不得跨越职责调用
3. 如果rush package修改了一些模块的具体实现，检查rush-code-design的SKILL是否同步，如果没有同步，需要同步对应更改

## 代码架构

1. `NavInterface`是专门和其他模块通信用的模块，所有ros2特有的功能集成在这里，日志相关的功能尽量集成在这个模块内，如果涉及到内部实现细节的日志另作讨论
2. `RushExtended`是集成`map`和`planner`的调用模块，只负责在接收消息后做的回应，以及掌控rush包的整体流程
3. `map`模块是专门负责感知的模块，分成静态地图和动态地图两块，静态地图负责从配置里读数据生成，而后不进行任何"写操作"; 动态地图则负责对机器人周边的情况进行更新; 对其他模块，占据地图通常只负责提供`是否被占据`和`ESDF值`两项数据，但是拓扑地图可能负责更多的版块，如边信息等
4. `planner`模块的任何类都只应对外提供一个接口，就是规划接口，可以在结果值内存储一些上层模块需要确认的信息，但是不建议对结果类型大规模扩展字段，或者说应该限制该结果类型字段数量
5. 其他模块大多是和`map`和`planner`平行的模块，负责处理比赛中可能出现的其他情况，统一受到`RushExtended`模块调度

## 更多信息

可以阅读`references`下的文件来了解更具体的模块职能和接口