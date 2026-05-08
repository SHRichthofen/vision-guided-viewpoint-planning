# View Goal Solver 硬约束推进失败专项调研

日期：2026-05-06

## 1. 问题重新定义

这次不再把重点放在“feasible 内如何排序”，而是直接看当前最常见失败：

```text
has_feasible_candidate: false
constraints_satisfied: false
```

旧 debug 中的典型超限：

```text
lateral_max: 0.023 ~ 0.032 m，limit = 0.020 m
axis_max:    0.534 ~ 0.694 rad，limit = 0.500 rad
```

也就是说，失败候选通常不是离谱，而是卡在硬约束边界外一点点。

当前参数中 `enable_fov_constraint: false`，所以旧 debug 里的 FOV 失败项在当前配置下不应再阻止发布。当前硬约束主要是：

```text
axis_error <= axis_max_rad
lateral_error <= lateral_max_m
axial_standoff in [standoff_min_m, standoff_max_m]
joint_limit_margin >= 0
```

## 2. 当前实现的本质

源码当前仍是：

```text
q seed
  -> coordinate descent
  -> FK(q)
  -> scalar weighted score
```

关键代码位置：

```text
src/control/src/view_goal_solver.cpp
ViewGoalSolver::optimize()
ViewGoalSolver::evaluate()
```

搜索器每次只试：

```text
q[j] + step
q[j] - step
```

并且只接受 `score.total` 更小的状态。

这意味着 optimizer 看到的是一个标量黑盒，不知道：

```text
axis_error 对哪些关节最敏感
lateral_error 应该沿哪个联合方向下降
当前失败主要是哪一个约束在阻止 feasible
多个关节是否必须同时动才能跨过边界
```

## 3. 当前 score 的问题不是“太短”，而是“不分层”

当前 `score.total` 混合了：

```text
gaze soft term
axis soft term
standoff soft term
lateral soft term
FOV soft term
motion soft term
joint limit soft term
wrist motion soft term
hard_constraint_weight * hard_penalty
```

这有两个后果。

第一，硬约束其实只是另一个权重项，不是真正的硬约束。

```text
如果某个候选稍微违反 axis/lateral，
但 motion/gaze/FOV/其他 soft 项更低，
coordinate descent 仍可能停在不可行点。
```

第二，hard penalty 的尺度会影响“硬”的程度。

例如 lateral hard penalty 使用的是 `lateral_scale`，而当前 `lateral_scale` 又受 standoff interval 影响，可能远大于 `lateral_max_m`。结果是几毫米的 lateral 超限在总分里未必足够强。

这不是单纯把 `hard_constraint_weight` 调大就能彻底解决的问题，因为高权重 penalty 仍然会带来：

```text
数值尺度难调
边界附近很陡/很平的混合区域
不同约束之间互相抢权重
局部搜索仍不知道联合下降方向
```

## 4. 信息是否不足？

分两类看。

### 4.1 推进 axis/lateral/standoff 的信息基本够

当前 solver 已经能从 FK 得到：

```text
camera_position
camera_z
axis_error
lateral_error
axial_standoff
```

这些足以定义观察几何约束。问题不是“完全没信息”，而是这些信息被压成了一个 `score.total`，丢掉了向量残差结构。

更适合求解器的信息形式应该是：

```text
r(q) =
[
  axis residual,
  lateral residual,
  standoff interval violation
]
```

而不是：

```text
score(q) = weighted sum of everything
```

### 4.2 判断真实可见性的信息不足

FOV 投影不包含圆柱侧壁遮挡，因此它不适合作为“看清管底”的硬约束。这一点和之前 survey 结论一致。

但当前最直接的 hard constraint 失败，在 FOV 关闭后主要不是可见性建模问题，而是 axis/lateral 约束求解问题。

### 4.3 规划成功信息也不足

solver debug 中：

```text
plan_success_checked: false
collision_checked: false
```

因此 solver 内部的 feasible 只表示几何与关节限位可行，不表示 MoveIt 路径一定能规划成功。

这属于第二阶段问题；当前第一阶段先解决几何 feasible。

## 5. 求解器设计是否不够好？

是。比 score 表达更核心。

### 5.1 Coordinate descent 对耦合约束很弱

axis/lateral 是由多个关节共同决定的非线性函数。很多情况下，单独动任意一个关节都会让总分变差，只有多个关节一起动才会降低约束违反量。

当前 coordinate descent 的接受准则是：

```text
单关节试探必须立即降低 total score
```

这会让搜索很容易停在“单轴方向都不改善，但联合方向能改善”的点。

这和手动调整体验一致：人通常会同时改肩、肘、腕的组合姿态，而不是严格一次只改一个关节。

### 5.2 Multi-start 不能替代方向信息

当前 seeds 是从 `q0` 做确定性扰动和一个 centered seed。它能扩大一点覆盖，但不能告诉优化器“怎么向约束边界内走”。

如果目标位姿每次不同，又不能使用人工示教 seed，那么靠固定 seed pattern 很难稳定覆盖所有目标。

### 5.3 当前方法比 IK/SQP 还少一层关键能力

TRAC-IK 的论文和 MoveIt 文档都强调：KDL 这类数值 IK 会受 joint limits、local minima、seed 影响而出现 false negatives；TRAC-IK 通过随机跳出局部极小和 SQP 形式改善这一点。当前 solver 甚至不是 Jacobian IK，只是黑盒 coordinate descent，所以遇到类似问题并不意外。

## 6. 文献/工具启发

### 6.1 OMPL / MoveIt constrained planning

MoveIt 的 constrained planning 文档明确把小体积甚至零体积的 Cartesian 约束区域作为适用场景，因为普通采样或普通状态空间搜索很难稳定命中这类区域。

对当前问题的启发：

```text
axis/lateral 约束本质上就是很窄的任务空间区域。
如果继续在 q-space 做普通局部搜索，很容易贴着边界外停住。
```

### 6.2 TRAC-IK

TRAC-IK 对 KDL 的问题判断和当前现象相似：

```text
joint limits 下容易 false negative
local minima 会让数值法卡住
需要随机重启 / SQP / 更好的约束处理
```

对当前问题的启发：

```text
问题不一定是可行解不存在，而是求解器没有足够能力找到。
```

### 6.3 TrajOpt / sequential convex optimization

TrajOpt 使用 sequential convex optimization 和 hinge loss 处理约束/碰撞，并通过外层逐步提高 penalty。关键不是“用了 hinge”，而是：

```text
它是带局部模型的优化，不是单纯 weighted sum + 坐标试探。
```

对当前问题的启发：

```text
如果要用 penalty，也应该服务于一个 feasibility-first 或 sequential optimizer，
而不是继续堆到一个总分里让 coordinate descent 猜方向。
```

### 6.4 pick_ik / IK cost functions

MoveIt 的 pick_ik 文档把 IK 求解拆成 global optimizer + local optimizer，并区分 position/orientation threshold、cost threshold、minimal displacement weight。

MoveIt 的 kinematics cost function 文档也提醒：准确性和额外 cost function 之间存在 tradeoff。

对当前问题的启发：

```text
先满足 pose/constraint threshold，再谈 motion cost。
不要在未满足硬约束时让 minimal motion 这类偏好项参与主导。
```

## 7. 对当前方案的判断

### 7.1 当前函数表达：不是冗长本身，而是职责混杂

当前 score 同时承担：

```text
1. 找 feasible
2. 选舒服姿态
3. debug/解释观察质量
```

这三个职责不应该用一个标量完成。

更合理的是：

```text
feasibility objective:
  只负责把 hard constraint violation 变成 0

quality objective:
  只在 feasible 以后用于选优

debug metrics:
  可以丰富，但不应该都进入优化目标
```

### 7.2 当前信息：几何求解够用，但缺少残差向量/方向

axis/lateral/standoff 信息够定义目标，但 optimizer 应看到残差向量，而不是只看总分。

### 7.3 当前求解器：最大短板

最大问题很可能是：

```text
q-space coordinate descent + scalar weighted score
```

而不是某个参数或某个 cost 项。

## 8. 推荐的下一步，不是大改采样

不建议马上上大规模 Cartesian pose sampling。你担心它和机器人运动模型脱节，这个担心成立。

更推荐从当前架构最小转向：

### Step 1：增加纯诊断，不改行为

在 debug 中明确输出每个 hard constraint 的归一化 violation 和一个总 violation：

```text
axis_violation_norm
lateral_violation_norm
standoff_violation_norm
hard_violation_total
```

目的不是优化，而是确认失败到底是：

```text
axis 主导
lateral 主导
二者互相牵制
还是被其他 soft 项吸走
```

### Step 2：增加 feasibility-only 搜索模式

不要 deadband，不要 rank。

当还没有 feasible candidate 时，优化目标临时切成：

```text
minimize hard_violation_total
```

不加入：

```text
motion_cost
wrist_motion_cost
gaze soft preference
FOV soft preference
joint limit comfort
```

只有 `hard_violation_total == 0` 后，再用原始 `score.total` 或一个简单 quality score 选发布目标。

这比继续调权重更简单，也更符合当前问题。

### Step 3：如果 coordinate descent 仍失败，换“方向型”局部求解

优先考虑有限差分 Jacobian / damped least squares：

```text
r(q) = active hard constraint residual vector
J = finite_difference(r, q)
delta_q = - (J^T J + lambda I)^-1 J^T r
q <- q + clamp(delta_q)
```

这不是手动 seed，也不是盲 pose sampling。它仍然从当前目标的几何误差出发，利用机器人 FK 模型自动找联合关节更新方向。

这一步比 IK 采样更贴近当前 solver：

```text
仍然 q -> FK -> residual
但从标量黑盒搜索变成残差向量优化
```

### Step 4：再考虑 IK 或 constrained planning

如果 Step 3 仍不稳，再考虑：

```text
TRAC-IK / pick_ik
MoveIt constrained planning
小规模任务区域 candidate + IK
```

但这应是后续，不是现在第一刀。

## 9. 最短结论

当前问题更像：

```text
求解器设计不适合硬约束推进
>
score 表达略复杂
>
信息不足
```

下一版不应继续扩写 weighted score。更应该做：

```text
feasibility-first objective
active hard constraint residual
方向型局部更新
```

如果只做一件事，建议先做：

```text
当 has_feasible_candidate=false 时，只优化 hard_violation_total。
```

这一步足够小，也能最快验证“失败是权重混杂导致，还是 coordinate descent 本身不够”。

## 10. 参考来源

- MoveIt, Using OMPL Constrained Planning: https://moveit.picknik.ai/main/doc/how_to_guides/using_ompl_constrained_planning/ompl_constrained_planning.html
- OMPL, Constrained Planning documentation: https://docs.ros.org/en/iron/p/ompl/doc/markdown/constrainedPlanning.html
- Beeson & Ames, TRAC-IK: An Open-Source Library for Improved Solving of Generic Inverse Kinematics, Humanoids 2015: https://doi.org/10.1109/HUMANOIDS.2015.7363472
- MoveIt, TRAC-IK Kinematics Solver: https://moveit.picknik.ai/main/doc/how_to_guides/trac_ik/trac_ik_tutorial.html
- Schulman et al., Motion planning with sequential convex optimization and convex collision checking, IJRR 2014: https://journals.sagepub.com/doi/10.1177/0278364914528132
- MoveIt, pick_ik Kinematics Solver: https://moveit.picknik.ai/main/doc/how_to_guides/pick_ik/pick_ik_tutorial.html
- MoveIt, Kinematics Cost Functions: https://moveit.picknik.ai/main/doc/how_to_guides/kinematics_cost_function/kinematics_cost_function_tutorial.html
