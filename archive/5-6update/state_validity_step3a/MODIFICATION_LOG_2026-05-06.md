# State Validity Step 3A 修改记录

日期：2026-05-06

## 修改目标

落实 3A：

```text
在 view_goal_solver 发布 q_goal 前，调用 MoveIt /check_state_validity。
只有 geometry feasible 且 MoveIt state valid 的候选才作为发布目标。
```

这一步针对当前问题：

```text
solver 找到了 constraints_satisfied=true 的 q_goal，
但 MoveIt 报 Unable to sample any valid states for goal tree。
```

也就是“观察几何可行，但 planning scene state invalid”。

## 修改文件

```text
src/control/src/view_goal_solver.cpp
src/control/config/view_goal_solver.params.yaml
src/control/CMakeLists.txt
src/control/package.xml
```

## 实现方式

### 1. 使用 MoveIt GetStateValidity 服务

新增依赖：

```text
moveit_msgs
```

solver 创建 service client：

```text
/check_state_validity
```

请求内容：

```text
group_name: arm
robot_state.is_diff: true
robot_state.joint_state.name: joint_names
robot_state.joint_state.position: candidate.q
```

`is_diff=true` 的目的是只覆盖 arm 关节，其他非 arm 关节沿用当前 planning scene 状态。

### 2. 不在每个 trial 中检查

state validity 是离散且较重的 planning scene 查询，因此本次不放进 `evaluate()`，也不进入每个 coordinate descent trial。

当前流程：

```text
1. coordinate descent 继续搜索几何 feasible candidate。
2. 搜索过程中收集 top-N geometry feasible candidates。
3. optimize 结束后，按 quality_score 从好到差调用 /check_state_validity。
4. 第一个 valid candidate 成为 best_valid_feasible。
5. 如果全部 invalid，则不发布，并输出 contacts/debug。
```

### 3. 候选选择层级

新增选择优先级：

```text
best_valid_feasible:
  constraintsSatisfied && state_valid
  用于发布

best_checked_feasible:
  constraintsSatisfied && 已检查但 state invalid
  用于 debug

best_feasible:
  constraintsSatisfied
  几何可行，但可能没通过 state validity

best_overall:
  hard_violation_total 最低
```

发布条件新增 state validity gate：

```text
constraints_ok && state_valid
```

如果 `require_state_validity_for_publish=true`，几何可行但 state invalid 的目标不会发布给 executor。

## 新增参数

`src/control/config/view_goal_solver.params.yaml`：

```yaml
enable_state_validity_check: true
require_state_validity_for_publish: true
state_validity_service: /check_state_validity
state_validity_service_wait_sec: 0.05
state_validity_response_timeout_sec: 0.25
state_validity_max_checks: 12
state_validity_candidate_pool_size: 32
state_validity_max_contacts: 6
```

含义：

```text
enable_state_validity_check:
  是否启用 /check_state_validity。

require_state_validity_for_publish:
  是否要求 state valid 才能发布。

state_validity_max_checks:
  每次 solve 最多检查多少个 geometry feasible candidates。

state_validity_candidate_pool_size:
  搜索过程中保留多少个 geometry feasible candidates。

state_validity_max_contacts:
  debug 中最多记录多少个 contact pair。
```

## Debug 新增字段

新增：

```text
has_state_valid_feasible_candidate
best_feasible_state_valid
state_validity_enabled
state_validity_required_for_publish
state_validity_service_available
state_validity_check_count
state_validity_invalid_count
state_validity_error_count
state_validity_failures
state_validity_checked
state_valid
state_validity_error
state_validity_contacts
```

如果 q_goal invalid，重点看：

```text
state_validity_contacts
state_validity_failures[].contacts
```

其中的 `body_1/body_2` 会说明是自碰还是碰环境。

## 回退方式

完全关闭检查：

```yaml
enable_state_validity_check: false
```

只保留 debug 检查、不阻止发布：

```yaml
require_state_validity_for_publish: false
```

## 本次本地验证

已执行：

```bash
colcon build --packages-select control
```

结果：

```text
1 package finished
```

## 预期现场表现

如果之前的 target 32 q_goal 本身 invalid：

```text
has_feasible_candidate: true
has_state_valid_feasible_candidate: false 或 true
state_validity_check_count > 0
state_validity_invalid_count > 0
state_validity_contacts: [...]
```

如果 top-N 中找到 valid 替代候选：

```text
selected_candidate: best_valid_feasible
published: true
```

如果 top-N 全 invalid：

```text
selected_candidate: best_feasible_state_invalid
published: false
selected_reason: state_validity_failed
```

这时应根据 contact pair 判断是 SRDF collision matrix 问题、夹爪/腕部自碰，还是环境碰撞。
