# View Goal Solver Feasibility-First 优化方案

日期：2026-05-06

## 1. 核心结论

基于 `VIEW_GOAL_HARD_CONSTRAINT_SOLVER_SURVEY_2026-05-06.md`，当前最值得优先处理的不是
feasible 内排序，也不是继续加大 `hard_constraint_weight`，而是把 solver 的第一目标改成：

```text
先找到满足硬约束的 q，再在可行解里选舒服姿态。
```

当前常见失败是：

```text
has_feasible_candidate: false
constraints_satisfied: false
```

并且超限通常很小：

```text
lateral_max: 0.023 ~ 0.032 m, limit = 0.020 m
axis_max:    0.534 ~ 0.694 rad, limit = 0.500 rad
```

这说明问题更像“局部搜索没有稳定推入窄可行域”，而不是“可行解明显不存在”。

## 2. 不建议作为第一刀的方向

### 2.1 不继续单纯调权重

`hard_constraint_weight` 再大，本质上仍是 weighted score 里的一项。它不能解决：

```text
soft objective 稀释 hard violation
不同约束尺度互相抢权重
coordinate descent 缺少联合下降方向
```

### 2.2 不优先做 deadband/ranking

deadband 和 feasible ranking 只在已经找到可行解时有价值。当前瓶颈是常常没有
`best_feasible`，因此第一阶段应先提升 feasible 命中率。

### 2.3 不立即跳到大规模 Cartesian sampling

Cartesian candidate + IK 可以作为中期方案，但它会引入更多新变量：

```text
候选姿态采样密度
roll 采样
IK 插件行为
候选与 MoveIt 规划成功率的关系
```

当前更小的转向是：仍保留 `q -> FK -> evaluate`，但把优化目标拆成 feasibility 与 quality。

## 3. 新目标分层

### 3.1 Constraint violation

在 `Score` 中增加硬约束归一化违反量：

```text
axis_violation_norm
lateral_violation_norm
standoff_min_violation_norm
standoff_max_violation_norm
gaze_violation_norm
fov_violation_norm
bottom_rim_fov_violation_norm
joint_limit_violation_norm
hard_violation_total
```

其中当前配置下 FOV 关闭，因此 FOV violation 应为 0，不参与发布阻断。

推荐归一化尺度：

```text
axis_scale = max(0.05 rad, axis_max_rad)
lateral_scale = max(0.005 m, lateral_max_m)
standoff_scale = max(0.03 m, standoff_max_m - standoff_min_m)
gaze_scale = max(0.05 rad, gaze_max_rad)
```

定义：

```text
axis_violation_norm =
  enforce_axis_max ? max(0, axis_error - axis_max_rad) / axis_scale : 0

lateral_violation_norm =
  enforce_lateral_max ? max(0, lateral_error - lateral_max_m) / lateral_scale : 0

standoff_min_violation_norm =
  max(0, standoff_min_m - axial_standoff) / standoff_scale

standoff_max_violation_norm =
  max(0, axial_standoff - standoff_max_m) / standoff_scale
```

`hard_violation_total` 用平方和即可：

```text
hard_violation_total =
  axis_violation_norm^2
+ lateral_violation_norm^2
+ standoff_min_violation_norm^2
+ standoff_max_violation_norm^2
+ optional enabled hard terms
```

### 3.2 Feasibility objective

当候选还没有满足硬约束时，优化器只看：

```text
minimize hard_violation_total
```

此阶段不加入：

```text
motion_cost
wrist_motion_cost
joint_limit comfort
gaze soft preference
axis_alignment soft preference
lateral soft preference
FOV soft preference
```

### 3.3 Quality objective

只有候选已满足 `constraintsSatisfied(score)` 后，才进入 quality 排序：

```text
quality =
  1.0 * motion_cost
+ 1.0 * wrist_motion_cost
+ 2.0 * joint_limit_cost
+ 0.05 * axis_error^2
+ 0.05 * (lateral_error / lateral_max_m)^2
```

这不是最终权重，只是初始建议。重点是：quality 不能让已可行候选重新越过硬约束边界。

## 4. 第一阶段实现：Feasibility-first coordinate descent

保持现有 multi-start 和 coordinate descent 框架，但改接受准则。

当前接受准则：

```text
trial.score.total < candidate.score.total
```

建议改成：

```text
if candidate is not feasible:
  accept trial if trial.hard_violation_total is lower
  tie break by lower quality only when violation nearly equal

if candidate is feasible:
  accept trial only if trial is still feasible and trial.quality is lower
```

全局候选也拆成两类：

```text
best_overall:
  min hard_violation_total, 用于诊断不可行时离边界最近的候选

best_feasible:
  constraintsSatisfied && min quality, 用于发布
```

这样即使 soft score 更好，也不能压过“向硬约束内走”。

## 5. 第二阶段实现：Residual/DLS rescue

如果第一阶段 coordinate descent 仍没有找到 feasible，再启用一个小步数的方向型局部求解。

### 5.1 Residual vector

定义 active hard residual：

```text
r(q) =
[
  axis_hinge,
  lateral_hinge,
  standoff_min_hinge,
  standoff_max_hinge
]
```

其中：

```text
axis_hinge = max(0, axis_error - axis_max_rad) / axis_scale
lateral_hinge = max(0, lateral_error - lateral_max_m) / lateral_scale
standoff_min_hinge = max(0, standoff_min_m - axial_standoff) / standoff_scale
standoff_max_hinge = max(0, axial_standoff - standoff_max_m) / standoff_scale
```

如果后续想给 lateral 更多方向信息，可以把 lateral 从 1 维扩为 2 维：

```text
lateral_offset = dot(offset, u) * u + dot(offset, v) * v
lateral_overflow = max(0, norm(lateral_offset) - lateral_max_m)
r_lateral_uv = lateral_overflow * normalize([dot(offset,u), dot(offset,v)]) / lateral_scale
```

第一版先用 scalar hinge 更简单，有限差分 Jacobian 仍能给出下降方向。

### 5.2 Finite difference Jacobian

对每个关节做小扰动：

```text
J[:, j] = (r(q + eps * e_j) - r(q - eps * e_j)) / (2 * eps)
```

推荐初始参数：

```yaml
enable_residual_dls: true
residual_dls_iterations: 25
residual_dls_fd_eps_rad: 0.003
residual_dls_lambda: 0.02
residual_dls_max_step_rad: 0.10
residual_dls_line_search_shrink: 0.5
```

更新：

```text
dq = - (J^T J + lambda I)^-1 J^T r
q_next = clampToBounds(q + clamp_norm(dq, residual_dls_max_step_rad))
```

用 line search 接受：

```text
只接受 hard_violation_total 下降的 q_next
```

若某次 DLS 进入 feasible，则交回 quality coordinate descent 做少量舒适性 refine。

## 6. 推荐参数开关

新增参数建议：

```yaml
enable_feasibility_first: true
feasibility_epsilon: 1.0e-8
feasibility_tie_break_epsilon: 1.0e-6

enable_residual_dls: true
residual_dls_iterations: 25
residual_dls_fd_eps_rad: 0.003
residual_dls_lambda: 0.02
residual_dls_max_step_rad: 0.10
residual_dls_line_search_shrink: 0.5

quality_weight_motion: 1.0
quality_weight_wrist_motion: 1.0
quality_weight_joint_limit: 2.0
quality_weight_axis_residual: 0.05
quality_weight_lateral_residual: 0.05
```

旧 `score.total` 可以保留一版，用于兼容 debug 与回退；但发布选择不再依赖它。

## 7. Debug 输出

新增 debug 字段：

```text
optimization_strategy: "feasibility_first_coordinate_descent"
hard_violation_total
axis_violation_norm
lateral_violation_norm
standoff_min_violation_norm
standoff_max_violation_norm
quality_score
best_overall_hard_violation_total
best_feasible_quality_score
residual_dls_attempted
residual_dls_accepted_steps
residual_dls_final_violation
```

如果仍失败，debug 应能直接回答：

```text
是 axis 卡住
还是 lateral 卡住
还是 standoff 区间卡住
是否 DLS 有下降但没进 feasible
是否所有候选都被 joint limit 推到边界
```

## 8. 代码落点

主要修改 `src/control/src/view_goal_solver.cpp`：

```text
Score:
  增加 violation 与 quality 字段

evaluate():
  继续计算现有几何指标
  额外调用 computeConstraintViolation(score)
  额外调用 computeQualityScore(score)

optimize():
  改为 feasibility-first 接受准则
  每个 seed 结束后，如果没有 feasible，可选调用 residual DLS rescue

considerCandidate():
  best_overall 使用 hard_violation_total 比较
  best_feasible 使用 quality_score 比较

publishDebug():
  输出新增 violation / quality / DLS 字段
```

建议新增 helper：

```text
computeConstraintViolation(Score & score)
computeQualityScore(Score & score)
candidateBetterForSearch(trial, current)
considerFeasibilityFirstCandidate(candidate, result)
constraintResidual(q, ...)
residualDlsRefine(candidate, ...)
```

## 9. 迭代顺序

### Step 1：只加 violation debug，不改行为

目标：确认失败主要由 axis/lateral/standoff 哪项主导。

验收：

```text
debug 中能看到 hard_violation_total 和每项 normalized violation。
原始发布行为不变。
```

### Step 2：启用 feasibility-first coordinate descent

目标：验证“soft score 稀释 hard constraint”是否是主要原因。

验收：

```text
同一批目标下 has_feasible_candidate 比当前明显提高。
best_overall_constraint_failures 中轻微 axis/lateral 越界减少。
```

### Step 3：加入 residual DLS rescue

目标：处理 coordinate descent 单关节方向都不下降、但联合方向能下降的情况。

验收：

```text
residual_dls_attempted: true
residual_dls_accepted_steps > 0
hard_violation_total 有持续下降
部分原先失败目标变为 feasible
```

### Step 4：再考虑 Cartesian candidate / IK

只有当前三步仍不能稳定找到 feasible，再把旧 proposal 里的 Cartesian candidate sampler 作为中期方案引入。

## 10. 风险与回退

### 风险 1：只优化 hard violation 后动作变大

处理：

```text
只在未 feasible 时忽略 motion cost。
一旦 feasible，立刻切到 quality refine。
```

### 风险 2：DLS 有数值抖动

处理：

```text
默认小步长
line search 必须降低 hard_violation_total
每步 clampToBounds
失败则保留 coordinate descent 最佳候选
```

### 风险 3：进入 feasible 后贴边

处理：

```text
quality 中加入极小 residual preference
或新增 feasibility_margin，用 slightly-inside 作为发布偏好
```

## 11. 最短推荐

如果只做最小有效改动：

```text
1. 在 Score 中加入 hard_violation_total。
2. 当 candidate 不可行时，coordinate descent 只最小化 hard_violation_total。
3. best_feasible 用 quality_score 排序，best_overall 用 hard_violation_total 排序。
```

这一步足够小，能直接验证 survey 的核心判断：

```text
当前失败是否主要来自 weighted score 与硬约束推进职责混杂。
```
