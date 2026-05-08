# View Goal Solver Deadband 与可行解排序回退记录

日期：2026-05-06

## 1. 状态

本文件原本记录了一次尝试：

```text
1. axis/lateral soft objective 改为 deadband/hinge。
2. best_feasible 使用独立 feasible_rank。
3. 非 feasible 阶段增加 constraint_violation。
```

该尝试已回退，当前 `src/control/src/view_goal_solver.cpp` 已恢复到修改前的评分与候选选择逻辑。

## 2. 回退原因

旧 debug 显示当前最直接的问题是：

```text
has_feasible_candidate: false
constraints_satisfied: false
```

也就是说，系统常常还没有找到满足硬约束的候选。此时优先讨论 feasible 内排序或 deadband，会把问题带向“已有可行解后如何选优”，而不是当前真正的瓶颈：

```text
为什么 q-space coordinate descent 没有稳定推进到硬约束内？
```

## 3. 当前代码状态

当前 solver 仍保持原始结构：

```text
q seed -> coordinate descent -> FK -> weighted score
```

`best_overall` 与 `best_feasible` 仍使用同一个 `score.total` 比较：

```text
best_overall: min(score.total)
best_feasible: constraintsSatisfied && min(score.total)
```

axis/lateral soft cost 仍为修改前形式：

```text
axis_alignment = 1 - dot(camera_z, view_axis)
lateral_cost = (lateral_error / lateral_scale)^2
```

## 4. 后续方向

下一步不再继续扩展 feasible ranking，而是重新 survey：

```text
1. 当前约束表达是否给了 optimizer 足够可用的信息？
2. weighted score 是否让硬约束推进目标被其他项稀释？
3. coordinate descent 是否本身不适合这个约束几何？
4. 是否需要更简单的 feasibility-first 求解器，而不是更复杂的 score？
5. 是否应直接从机器人运动模型、Jacobian、IK 或 constrained optimization 角度重构搜索？
```
