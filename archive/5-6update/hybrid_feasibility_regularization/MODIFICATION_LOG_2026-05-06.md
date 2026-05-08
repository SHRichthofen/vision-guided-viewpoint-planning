# Hybrid Feasibility Regularization 修改记录

日期：2026-05-06

## 修改目标

把当前“纯 hard violation 推进”改成带运动学正则的 feasibility-first：

```text
不可行阶段：
  hard_violation_total 仍是主目标，但小幅 progress / tie 时用运动学舒适度约束搜索。

可行阶段：
  加强 motion / wrist / joint limit 权重，并加入 axis/lateral 余量偏好。
```

目标是减少几何可行但 MoveIt 不喜欢的边界姿态，同时保留 feasibility-first 的硬约束推进能力。

## 修改文件

```text
src/control/src/view_goal_solver.cpp
src/control/config/view_goal_solver.params.yaml
```

## 行为变化

### 1. Score 新增字段

新增：

```text
comfort_score
feasibility_merit
axis_margin_cost
lateral_margin_cost
```

含义：

```text
comfort_score = motion_cost + wrist_motion_cost + joint_limit_cost

feasibility_merit =
  hard_violation_total + infeasible_comfort_weight * comfort_score
```

### 2. 不可行阶段接受准则

旧逻辑：

```text
candidate 不可行时，只要 hard_violation_total 下降就接受。
```

新逻辑：

```text
如果 trial 进入 feasible：
  接受。

如果 hard_violation_total 明显下降：
  接受。

如果 hard_violation_total 只是小幅变化或近似打平：
  比较 feasibility_merit，选择更舒适的姿态。
```

这样避免为了极小的 violation 改善而大幅折腕或远离当前关节状态。

### 3. 可行阶段 quality 加强

`quality_score` 改为：

```text
quality_weight_motion * motion_cost
+ quality_weight_wrist_motion * wrist_motion_cost
+ quality_weight_joint_limit * joint_limit_cost
+ quality_weight_axis_residual * axis_margin_cost
+ quality_weight_lateral_residual * lateral_margin_cost
```

其中：

```text
axis_margin_cost:
  axis_error 越贴近 axis_max_rad，代价越高。

lateral_margin_cost:
  lateral_error 越贴近 lateral_max_m，代价越高。
```

这会让 feasible 内排序更偏向有余量、运动更小、腕部更温和的解。

### 4. State validity timeout 处理

参数中把：

```yaml
state_validity_response_timeout_sec: 0.25
```

调整为：

```yaml
state_validity_response_timeout_sec: 0.8
```

用于减少 `/check_state_validity` 偶发超时造成的误判式不发布。

## 新增/调整参数

```yaml
infeasible_comfort_weight: 0.08
infeasible_hard_progress_epsilon: 1.0e-6
quality_weight_motion: 2.0
quality_weight_wrist_motion: 2.0
quality_weight_joint_limit: 3.0
quality_weight_axis_residual: 0.15
quality_weight_lateral_residual: 0.10
state_validity_response_timeout_sec: 0.8
```

## Debug 新增字段

新增：

```text
feasibility_merit
comfort_score
axis_margin_cost
lateral_margin_cost
```

`state_validity_failures` 中也补充：

```text
comfort_score
feasibility_merit
```

用于解释为什么某个候选被排序到前面或被过滤。

## 本次本地验证

已执行：

```bash
colcon build --packages-select control
```

结果：

```text
1 package finished
```

## 现场观察重点

重点对比修改前后的：

```text
axis_error
motion_cost
wrist_motion_cost
comfort_score
has_state_valid_feasible_candidate
state_validity_error_count
```

预期：

```text
axis_error 不再长期贴着 0.4999x。
motion_cost / wrist_motion_cost 不再明显增大。
state_validity timeout 减少。
MoveIt 更少收到激进边界姿态。
```
