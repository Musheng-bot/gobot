# Gobot

我们的目标，是星辰大海

1. **Semantic Navigation / ObjectNav**
   目标：**“在未知房间里找到椅子并导航到它附近。”**
   重点：目标检测、语义地图、主动探索。

2. **Visual Language Navigation（VLN）**
   目标：**“走出房间，右转，找到走廊尽头的沙发。”**
   重点：语言 grounding、视觉历史、路径决策。

3. **Embodied Agent / Task Planning**
   目标：**“找到红色箱子，如果路径被挡住就换路，到达后继续执行下一任务。”**
   重点：VLM/LLM + Behavior Tree / Skill planning + 失败恢复。

4. **Learning-based Policy**
   目标：**“不用手写 frontier 规则，让网络自己学习下一步应该探索哪里。”**
   重点：Imitation Learning / RL、policy learning。

5. **World Model / Model-Based RL**
   目标：**“机器人先预测不同动作未来会发生什么，再选择最优动作。”**
   重点：学习动力学、latent world model、planning/MPC。

6. **VLA / 通用具身模型**
   目标：**“输入一句自然语言，让模型同时理解环境并完成导航或操作任务。”**
   重点：Vision-Language-Action、多任务数据、泛化。

主线是：

**ObjectNav → VLN → Embodied Agent → World Model → VLA**

