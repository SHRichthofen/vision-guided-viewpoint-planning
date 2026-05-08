# 机械臂观察位姿优化相关方法调研

日期：2026-05-06

## 1. 当前问题抽象

当前任务不是普通的“到达一个固定末端位姿”，而是：

```text
给定圆柱中心 C 和轴向 a，
寻找一个机械臂相机位姿 q，
使相机能够沿圆柱轴线附近观察管内/底部，
同时满足关节限位、可达性和执行舒适性。
```

这类问题更接近：

```text
workspace goal region / task-space region
pose-constrained manipulation planning
view planning / next-best-view
视觉闭环伺服
```

而不是单一点 IK。

## 2. Workspace Goal Regions / Task Space Regions

### 2.1 Workspace Goal Regions

Berenson 等人的 Workspace Goal Regions (WGRs) 将目标末端位姿定义为工作空间中的区域，而不是离散的单个关节目标。论文中使用随机规划算法，并将 RRT 探索和 Jacobian-based gradient descent 或 IKBiRRT 结合，以从区域目标中寻找路径。

这与当前问题高度相关：圆柱观察目标天然不是一个唯一位姿，而是一个“相机中心在圆柱轴线附近、姿态接近轴向”的可接受区域。

对当前任务的启发：

```text
不要追求一个 desired pose。
定义一个观察可接受区域。
在区域内选择最容易执行/最舒服的解。
```

来源：

- Berenson et al., Manipulation Planning with Workspace Goal Regions, ICRA 2009: https://publications.ri.cmu.edu/manipulation-planning-with-workspace-goal-regions

### 2.2 Task Space Regions

Task Space Regions (TSRs) 进一步将末端约束表示为 SE(3) 中的可采样区域，并提供快速距离度量。论文强调 TSR 适合采样式规划，因为可以直观指定约束、采样目标区域、计算到区域的距离。

对当前任务的启发：

```text
相机观察位姿可以被写成 TSR：
- 位置：沿圆柱轴向的一段区间 + 横向半径容差
- 姿态：相机 z 轴与圆柱轴线夹角小于阈值
- roll：可自由或弱约束
```

这比当前单一 weighted score 更稳定，因为它先表达“什么是可接受集合”，再在集合内优化运动舒适度。

来源：

- Berenson, Srinivasa, Kuffner, Task Space Regions: A Framework for Pose-Constrained Manipulation Planning, IJRR 2011: https://publications.ri.cmu.edu/task-space-regions-a-framework-for-pose-constrained-manipulation-planning

## 3. Constrained Motion Planning

OMPL 的 constrained planning 将任务约束写成：

```text
f(q) = 0
```

并在约束流形上采样/投影。官方文档指出，这类约束对低体积甚至零体积的笛卡尔约束区域尤其有价值，因为普通 rejection sampling 很难采到有效点。

MoveIt 2 的 constrained planning 文档也强调：

```text
对于小体积笛卡尔约束、平面/线约束、姿态约束，
应考虑 constrained state space / projection-based sampling。
```

对当前任务的启发：

```text
如果 axis/lateral 约束很紧，
单纯 q-space coordinate descent 很容易局部最优。
应考虑：
1. 显式构造约束区域；
2. 从区域内采样目标位姿；
3. 对每个目标位姿求 IK；
4. 再选择可规划的最好解。
```

来源：

- OMPL Constrained Planning documentation: https://docs.ros.org/en/iron/p/ompl/doc/markdown/constrainedPlanning.html
- MoveIt 2 Using OMPL Constrained Planning: https://moveit.picknik.ai/main/doc/how_to_guides/using_ompl_constrained_planning/ompl_constrained_planning.html

## 4. IK 求解器与局部最优

TRAC-IK 论文针对 KDL IK 的问题进行了系统讨论：数值 IK 容易出现 joint limits 下的 false negatives，容易卡在局部最小值，并且对笛卡尔位姿容差支持不足。TRAC-IK 的思路是结合改进的 Jacobian 方法和 SQP 优化，并并行运行多个策略以提高求解率。

MoveIt 的 IKFast 文档则强调解析 IK 的稳定和速度优势：IKFast 可以生成 C++ 解析求解器，通常微秒级返回解。

对当前任务的启发：

```text
当前 q-space coordinate descent 是局部搜索。
如果手动指引能到达更好位姿，但自动求解找不到，
问题很可能不是目标函数本身，而是 seed 和 IK 搜索不足。
```

可选方向：

```text
1. 使用多个 Cartesian goal samples + IK 求解。
2. 引入 TRAC-IK / IKFast / MoveIt setFromIK 多 seed。
3. 使用历史成功姿态或手动示教姿态作为 seed。
```

来源：

- Beeson & Ames, TRAC-IK: An Open-Source Library for Improved Solving of Generic Inverse Kinematics, Humanoids 2015: https://www.researchgate.net/publication/282852814_TRAC-IK_An_Open-Source_Library_for_Improved_Solving_of_Generic_Inverse_Kinematics
- MoveIt IKFast documentation: https://moveit.picknik.ai/main/doc/examples/ikfast/ikfast_tutorial.html

## 5. View Planning / Next-Best-View

主动视觉与 NBV 文献通常将问题分成：

```text
候选视点生成
候选视点评分
可达性/运动约束过滤
执行或闭环更新
```

View planning survey 指出，视点规划质量受任务、硬件条件、扫描状态和规划策略共同影响。NBV 论文中常见做法是先生成 reachable candidate views，再基于可见性、信息增益、遮挡或模型投影进行选择。

对当前任务的启发：

```text
FOV/投影更适合用于候选视点评分或粗过滤，
不应单独代表真实可见性。
```

当前实验已经证明：虚拟底部 rim 可以投进图像，但真实底面仍会被圆柱侧壁遮挡。因此对于管内观察，FOV 投影缺少遮挡模型，不适合作为主约束。

来源：

- Zeng et al., View planning in robot active vision: A survey of systems, algorithms, and applications, Computational Visual Media 2020: https://link.springer.com/article/10.1007/s41095-020-0179-3
- McGreavy, Kunze, Hawes, Next Best View Planning for Object Recognition in Mobile Robotics, PlanSIG 2016: https://www.research.ed.ac.uk/en/publications/03497b24-fa07-42de-93a9-1858dc78fa79
- Vasquez-Gomez et al., Volumetric Next-best-view Planning for 3D Object Reconstruction with Positioning Error, IJARS 2014: https://journals.sagepub.com/doi/abs/10.5772/58759

## 6. Visual Servoing

Chaumette 与 Hutchinson 的视觉伺服综述将视觉伺服分为 Image-Based Visual Servoing (IBVS) 和 Position-Based Visual Servoing (PBVS)。核心思想是把视觉信息放进控制回路，而不是只执行一次离线预测的位姿。

对当前任务的启发：

```text
执行前 solver 只需给出一个几何上合理的初始观察位姿。
真正的“管底居中/看清”可以通过执行后小范围视觉闭环修正。
```

这对当前系统尤其重要，因为：

```text
检测噪声、手眼标定误差、圆柱轴线估计误差、机械臂执行误差
都会使一次性预测位姿偏离真实最优。
```

来源：

- Chaumette & Hutchinson, Visual Servo Control. I. Basic Approaches, IEEE Robotics & Automation Magazine 2006: https://experts.illinois.edu/en/publications/visual-servo-control-i-basic-approaches

## 7. 对当前 solver 的客观结论

### 7.1 当前 coordinate descent 的问题

当前 solver 在 q-space 上做 coordinate descent：

```text
q -> FK(q) -> score
```

优点：

```text
实现简单
不依赖 IK 插件
可以直接评价相机姿态
```

缺点：

```text
局部搜索强
seed 覆盖不足时容易找不到手动可达的更好点
weighted sum 权重难调
约束很紧时可行域窄，坐标下降很难碰到 feasible
```

### 7.2 FOV 投影的定位

FOV 投影有用，但不应作为主约束：

```text
适合作为明显出画的粗检查和 debug
不适合判断管底真实可见性
不能处理圆柱侧壁遮挡
```

### 7.3 更可靠的问题表达

更接近文献中 TSR/WGR 的表达方式：

```text
先定义观察可接受区域：
- axis_error <= axis_max
- lateral_error <= lateral_max
- axial_standoff in [min, max]

再在可接受区域内选择：
- 关节限位裕量大
- 运动距离小
- 腕部动作小
- 执行路径可规划
```

这比用一个总 score 混合所有项更稳。

