# View Goal Solver 下一版优化方案

日期：2026-05-06

## 1. 目标判断

当前实测说明：

```text
FOV 虚拟投影不能代表管底真实可见性。
standoff 不应成为主目标。
axis 与 lateral 才是观察几何的核心。
coordinate descent 容易卡局部最优。
```

因此下一版 solver 不应继续沿着“把更多项塞进 weighted score”推进，而应改成：

```text
几何可行域 + 可行域内排序 + 更强 seed/采样
```

## 2. 推荐的新问题定义

### 2.1 主几何约束

定义：

```text
C = cylinder top center
a = view_axis
p = camera position
z = camera optical axis

axial = dot(p - C, -a)
lateral = norm((p - C) - axial * (-a))
axis_error = angle(z, a)
```

主约束：

```text
axis_error <= axis_max_rad
lateral <= lateral_max_m
axial in [standoff_min_m, standoff_max_m]
joint limits valid
```

注意：`standoff_min_m` 只防止极端贴近/碰撞，不代表“越高越好”。

### 2.2 FOV 角色

FOV 不再作为观察质量核心，只做：

```text
明显出画检查
debug 辅助解释
后续可选保护
```

推荐短期设置：

```yaml
enable_fov_constraint: false
```

或者保留但极宽：

```yaml
min_fov_score: 0.20
min_bottom_rim_fov_score: 0.20
weight_fov: 0.0 ~ 1.0
weight_bottom_rim_fov: 0.0 ~ 1.0
```

## 3. 把 axis/lateral 改成 deadband/hinge

当前 soft cost 会在满足约束后继续追求更小误差。更合理的形式是：

```text
axis_cost = max(0, axis_error - axis_good_rad)^2
lateral_cost = max(0, lateral_error - lateral_good_m)^2
```

如果不想新增参数，可以先复用：

```text
axis_good_rad = axis_max_rad
lateral_good_m = lateral_max_m
```

含义：

```text
进入可接受区间后不再强迫继续变小。
solver 可以把自由度留给关节限位、运动距离和执行稳定性。
```

这会减少“为了更小 axis/lateral，反而跑到难执行姿态”的情况。

## 4. 从 weighted sum 改为两层选择

### 4.1 第一层：找 feasible

定义约束违反量：

```text
V =
  h(axis_error - axis_max_rad)^2
+ h(lateral_error - lateral_max_m)^2
+ h(standoff_min_m - axial)^2
+ h(axial - standoff_max_m)^2
+ joint_limit_violation
```

其中：

```text
h(x) = max(0, x / scale)
```

第一阶段目标：

```text
尽可能让 V = 0
```

### 4.2 第二层：可行解内排序

当候选满足约束后，不再按原始总分选择，而按执行舒适性排序：

```text
rank =
  motion_cost
+ wrist_motion_cost
+ joint_limit_cost
+ small residual geometric preference
```

建议：

```text
best_feasible 的排序不要和 best_overall 完全共用 score。
best_overall 用于诊断。
best_feasible 用于发布。
```

当前已经实现 `best_overall / best_feasible`，下一步是让 `best_feasible` 使用独立 ranking。

## 5. 更换搜索策略：从 q-space 局部下降到候选视点采样

当前 coordinate descent 的最大限制是 seed 覆盖不足。建议新增一层 Cartesian candidate generation。

### 5.1 生成观察候选

围绕圆柱轴线生成相机位置：

```text
p_candidate =
  C
+ axial_i * (-a)
+ lateral_j * (cos(phi_j) * u + sin(phi_j) * v)
```

其中：

```text
axial_i: 在 [standoff_min_m, standoff_max_m] 内采样
lateral_j: 0 到 lateral_max_m 内采样
phi_j: 绕圆柱轴线采样
u/v: 垂直于 a 的基向量
```

相机姿态：

```text
camera_z ≈ a
roll 自由采样
```

为什么 roll 要采样：

```text
很多 6DoF 小机械臂的可达性强烈依赖腕部 roll。
同一个 camera_z，roll 不同，IK 可行性可能完全不同。
```

### 5.2 对每个候选求 IK

对每个 Cartesian pose：

```text
setFromIK / MoveIt IK / TRAC-IK / IKFast
```

使用多个 seed：

```text
当前 q
上一次成功 q
手动示教 q
按目标方位分类的模板 q
当前 coordinate-descent seeds
```

每个 IK 解再用当前 `evaluate()` 打分和约束检查。

### 5.3 局部 refine

对 IK 得到的候选，再运行少量 q-space refine：

```text
只优化 hinge violation 和执行舒适性
不要再用 FOV 强拉
```

这比直接从当前 q 做 coordinate descent 更有机会进入正确 basin。

## 6. 历史成功 seed / 手动 seed

如果手动指引能达到更好的观察点，说明可行解存在。应把人工经验显式喂给优化器。

建议建立：

```text
view_goal_seed_library.yaml
```

按目标方位分组：

```text
top_vertical
front_left
front_right
side_left
side_right
near_center
far_center
```

每组存：

```text
q_seed
适用 axis/center 条件
成功次数
最近成功时间
```

优化时：

```text
根据 target center 和 view_axis 选择若干 seed。
先用 seed 做 FK 检查，再做 local refine。
```

这通常比继续加权重更有效。

## 7. 与 MoveIt / OMPL 的关系

短期不必完全迁移到 MoveIt constrained planning，但可以借鉴其思想：

```text
把目标写成区域，而不是单点。
从区域采样，而不是只在 q-space 局部扰动。
```

中期可考虑：

```text
1. MoveIt OrientationConstraint: 约束相机 z 轴方向。
2. PositionConstraint: 近似相机中心区域。
3. OMPL constrained planning: 对窄约束区域使用投影式采样。
4. IKFast/TRAC-IK: 提高 IK 求解率。
```

但当前最小改动路线是：

```text
自定义 Cartesian candidate sampler + MoveIt RobotState/IK + 当前 debug/evaluate
```

## 8. 推荐迭代计划

### Step 1：关闭 FOV 硬约束

目的：先验证纯几何约束是否可行。

```yaml
enable_fov_constraint: false
enforce_axis_max: true
enforce_lateral_max: true
enforce_gaze_max: false
```

观察：

```text
has_feasible_candidate
selected_candidate
axis_error
lateral_error
axial_standoff
joint_limit_margin
```

### Step 2：axis/lateral soft cost 改 hinge

目的：进入可接受区后不再过度优化几何误差。

实现：

```text
axis_alignment cost -> max(0, axis_error - axis_max_rad)^2
lateral_cost -> max(0, lateral_error - lateral_max_m)^2
```

### Step 3：best_feasible 独立排序

目的：可行解中选择最容易执行的。

排序：

```text
motion_cost
wrist_motion_cost
joint_limit_cost
small residual axis/lateral
```

### Step 4：加入 Cartesian candidate sampler

目的：解决局部最优和 seed 覆盖不足。

候选：

```text
axis samples × lateral ring samples × roll samples
```

### Step 5：加入成功 seed 库

目的：利用手动指引和历史成功经验。

## 9. 预期 debug 变化

改造后理想 debug：

```text
has_feasible_candidate: true
selected_candidate: best_feasible
constraint_failures: []
axis_error <= axis_max_rad
lateral_error <= lateral_max_m
axial_standoff in interval
```

如果仍失败：

```text
只有 axis_max 失败：
  说明姿态可达性/腕部 roll/IK seed 是主问题。

只有 lateral_max 失败：
  说明相机中心到轴线区域采样不足，或 lateral_max 太紧。

joint_limit_margin 接近 0：
  说明目标对当前机械臂接近边界，应优先换 seed / roll / 轴向高度，而不是继续收紧约束。
```

## 10. 当前建议

我建议下一次代码修改优先做：

```text
1. 关闭 FOV 硬约束。
2. axis/lateral 改 hinge cost。
3. best_feasible 独立 ranking。
```

这三步对现有架构侵入较小，能最快验证“几何问题描述是否正确”。如果这之后仍明显不如手动示教，再做 Cartesian candidate sampler 和 seed library。

