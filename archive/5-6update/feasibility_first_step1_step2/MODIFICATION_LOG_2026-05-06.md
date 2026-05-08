# Feasibility-First Step 1/2 修改记录

日期：2026-05-06

## 修改目标

按照 `VIEW_GOAL_SOLVER_FEASIBILITY_FIRST_OPTIMIZATION_PLAN_2026-05-06.md` 的前两步推进：

```text
Step 1：增加 hard constraint violation debug，不依赖旧 total score 判断失败原因。
Step 2：启用 feasibility-first coordinate descent，在不可行阶段优先最小化 hard_violation_total。
```

本次未实现 residual/DLS rescue；那是下一阶段用于处理多关节联合方向的改动。

## 修改文件

```text
src/control/src/view_goal_solver.cpp
src/control/config/view_goal_solver.params.yaml
```

## 行为变化

### 1. Score 新增字段

新增字段：

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
quality_score
```

这些字段用于把硬约束失败拆开看，避免只看到一个混合后的 `score.total`。

### 2. 新增 violation / quality 计算

新增 helper：

```text
axisViolationScale()
gazeViolationScale()
lateralViolationScale()
standoffViolationScale()
computeConstraintViolation()
computeQualityScore()
```

`hard_violation_total` 使用归一化 violation 平方和。

`quality_score` 只用于可行解排序，初始包含：

```text
motion_cost
wrist_motion_cost
joint_limit_cost
axis residual preference
lateral residual preference
```

### 3. 搜索接受准则改变

旧逻辑：

```text
trial.score.total < candidate.score.total
```

新逻辑：

```text
if 当前 candidate 不可行:
  优先接受 hard_violation_total 更低的 trial
  violation 近似相等时，用 quality_score 打破平局

if 当前 candidate 已可行:
  只接受仍然可行且 quality_score 更低的 trial
```

### 4. best_overall / best_feasible 选择改变

开启 `enable_feasibility_first` 时：

```text
best_overall:
  hard_violation_total 最低的候选，用于不可行时诊断“离约束最近”的状态

best_feasible:
  constraintsSatisfied && quality_score 最低的候选，用于发布
```

关闭 `enable_feasibility_first` 时，回退到旧的 `score.total` 比较逻辑。

### 5. Debug 输出新增字段

新增 debug 字段：

```text
optimization_strategy
best_overall_hard_violation_total
best_feasible_quality_score
hard_violation_total
axis_violation_norm
lateral_violation_norm
standoff_min_violation_norm
standoff_max_violation_norm
gaze_violation_norm
fov_violation_norm
bottom_rim_fov_violation_norm
joint_limit_violation_norm
quality_score
```

## 新增参数

`src/control/config/view_goal_solver.params.yaml` 新增：

```yaml
enable_feasibility_first: true
feasibility_epsilon: 1.0e-8
feasibility_tie_break_epsilon: 1.0e-6
quality_weight_motion: 1.0
quality_weight_wrist_motion: 1.0
quality_weight_joint_limit: 2.0
quality_weight_axis_residual: 0.05
quality_weight_lateral_residual: 0.05
```

## 回退方式

如需现场快速回退搜索行为：

```yaml
enable_feasibility_first: false
```

这样会保留新增 debug 字段，但搜索接受准则与候选排序回到旧的 `score.total` 逻辑。

## 验证建议

启动后重点观察 `/view_goal_solver_debug`：

```text
optimization_strategy == "feasibility_first_coordinate_descent"
has_feasible_candidate 是否提高
hard_violation_total 是否比旧 best_overall 更接近 0
axis_violation_norm / lateral_violation_norm 谁主导失败
best_feasible_quality_score 是否非 null
```

如果仍然出现轻微越界但 DLS 未实现，下一步应进入 residual/DLS rescue。

## 本次本地验证

已执行：

```bash
colcon build --packages-select control
```

结果：

```text
1 package finished
```
