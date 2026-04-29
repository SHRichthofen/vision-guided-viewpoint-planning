# 2026-04-23 单次定位测试模式收口

## 背景

用户当前目标不是连续追踪或二次修正，而是直接测试“单次定位 -> 单次规划 -> 单次执行”的精度。

此前即使候选机制已删除，链路中仍残留两类会导致二次动作的机制：

1. `arm_pose_controller.py` 仍会在执行器 busy / idle 切换后重新触发目标生成。
2. `vision_moveit_executor.cpp` 仍保留 `queue_latest_when_busy`，会在当前执行完成后立刻接手最新 pending 目标。

这会让系统表现成“第一次到位后又立刻来一次”，不利于单次定位精度评估。

## 本次修改

### 1. 控制层改为严格单次发布

文件：
- `src/vision_arm_control/vision_arm_control/arm_pose_controller.py`
- `src/vision_arm_control/config/arm_pose_controller_bottom_scan.params.yaml`

调整：
- 删除 `publish_once_per_selection` 参数，行为固定为单次。
- 删除执行器状态订阅与 busy gate：
  - `exec_status_topic`
  - `busy_status_states`
- 删除发布节流与重复目标判定参数：
  - `min_publish_interval_sec`
  - `min_position_delta_m`
  - `min_orientation_delta_rad`
- 删除无实际意义的 `look_at_depth_m` 参数。
- 当前逻辑改为：
  - 收到新的 `/target_id` 后，清空旧 pose 缓存；
  - 只接受该目标的第一帧 `/cylinder_pose_base`；
  - 生成并发布一次目标位姿；
  - 本轮后续视觉更新全部忽略，直到下一个 `/target_id`。

## 2. 执行器改为严格单次接单

文件：
- `src/control/src/vision_moveit_executor.cpp`
- `src/control/config/vision_moveit_executor.params.yaml`

调整：
- 删除 `/target_pose` 订阅，仅保留 `/target_pose_stamped`。
- 删除 `queue_latest_when_busy` 和 pending request 逻辑。
- 删除目标去重/节流参数：
  - `min_target_interval_sec`
  - `min_position_delta_m`
  - `min_orientation_delta_rad`
- 当前逻辑改为：
  - 空闲时接收一个目标并执行；
  - 执行中若再收到新目标，直接忽略；
  - 不排队，不补发，不二次定位。

## 3. 几何策略进一步收简

控制层只保留最基础策略：

- `camera_position = center - view_axis * camera_standoff_m`
- `look_at_point = center`

即：
- 相机从管口中心沿朝外方向退开固定距离；
- 直接看向管口中心；
- 不再引入“看多深”这类对当前单次测试无帮助的额外参数。

## 预期行为

在目标静止场景下，链路应表现为：

1. `cylinder_detection` 选中目标。
2. `vision_to_arm_transform` 发布 base 系位姿。
3. `arm_pose_controller` 仅发布一次目标位姿。
4. `vision_moveit_executor` 仅规划执行一次。
5. 即使视觉后续继续更新，也不会触发第二次规划。

## 备注

本次调整的目的不是最终策略定型，而是为“单次定位精度验证”提供一个干净、可解释、无二次扰动的基线版本。
