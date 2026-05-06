# 2026-05-05 View Goal Solver Analysis and Tuning Notes

本文记录本轮围绕机械臂观察位姿优化器 `view_goal_solver` 的分析、代码优化和调参注意事项。重点聚焦新算法本身，暂不展开 `pick_ik` 切换。

## 1. 当前算法定位

当前 `view_goal_solver` 的核心思路已经从旧的单一 pose 生成：

```text
camera_position = center - view_axis * camera_standoff_m
look_at_point = center
roll = 由 world_z 参考轴固定
```

升级为关节空间目标求解：

```text
q -> FK(q) -> camera_position / camera_z -> cost / constraints -> q_goal
```

也就是说，优化变量是机械臂关节角 `q`，不是一个预先手写出来的相机 pose。相机 roll 由候选关节解的 FK 结果自然决定，而不是由固定 `roll_reference_axis` 单独决定。

主链路：

```text
/cylinder_semantics_base
  -> center_base, axis_base
  -> 当前关节 q0
  -> 多 seed 坐标下降
  -> 评价 q 对应的真实 camera_color_optical_frame 位姿
  -> 发布 /view_goal_joint_target
  -> vision_moveit_executor setJointValueTarget(q_goal)
```

## 2. 已排查的问题

### 2.1 MoveIt 当前状态获取失败

现象：

```text
Failed to fetch current robot state
Didn't receive robot state with recent timestamp
Requested time ... latest received state has time ...
```

日志里的 requested/latest 通常只差几毫秒，因此更像单线程 executor 阻塞造成的状态回调无法推进，而不是 AGX `/feedback/joint_states` 时间戳根本错误。

原因：

`view_goal_solver` 原先使用：

```cpp
rclcpp::spin(node);
```

当 `/cylinder_semantics_base` 回调里调用 `move_group_->getCurrentState(...)` 等待当前状态时，MoveIt 内部的 joint state monitor 回调可能无法在同一单线程 executor 中及时执行。

已优化：

- 将 solver 的语义输入回调放入独立 callback group。
- `main()` 改为 `MultiThreadedExecutor(..., 2)`。
- 初始化 MoveGroup 后提前调用 `move_group_->startStateMonitor(0.0)`。

预期效果：

`getCurrentState()` 等待状态时，MoveIt 内部 `/joint_states` 回调可以由另一条 executor 线程处理，避免状态时间卡在请求时间之前。

### 2.2 Debug 输出被截断

`/view_goal_solver_debug` 是 `std_msgs/String` JSON。`ros2 topic echo` 默认截断长字符串到 128 字符。

查看完整输出：

```bash
ros2 topic echo /view_goal_solver_debug --field data --full-length --once
```

或连续查看：

```bash
ros2 topic echo /view_goal_solver_debug --field data --full-length
```

## 3. 当前代价与约束含义

基本输入：

```text
C = 管口中心，base frame
a = view_axis_base，base frame，约定为管内方向
B = C + depth_proxy_m * a，管内代理点
p = camera position from FK(q)
z = camera optical +Z from FK(q)
```

主要误差：

```text
gaze_error = angle(z, B - p)
axis_error = angle(z, a)
distance_to_center = ||C - p||
standoff_error = distance_to_center - standoff_desired_m
```

含义：

- `gaze_error`：相机光轴是否看向管内代理点。
- `axis_error`：相机光轴是否接近管轴方向。
- `standoff_error`：相机距离管口中心是否接近期望距离。

当前重要硬约束：

```text
standoff_min_m <= distance_to_center <= standoff_max_m
gaze_error <= gaze_max_rad
axis_error <= axis_max_rad
lateral_error <= lateral_max_m
fov_score >= min_fov_score
joint_limit_margin >= 0
```

注意：

`gaze_max_rad` / `axis_max_rad` 是发布门槛和超限惩罚边界，不是直接要求优化器一定收敛到该值以下的强等式目标。若代价函数总分最低的 best 不满足硬约束，当前 solver 会拒绝发布，而不是自动发布第二好的 feasible 解。

## 4. 新增横向距离约束

为解决候选相机位置虽然 gaze/axis 近似满足，但 3D 站位偏离管轴外侧观察走廊的问题，本轮加入了横向偏移约束。

理想观察方向：

```text
ideal_camera_direction = -view_axis
```

相机相对管口中心：

```text
center_to_camera = p_cam - C
```

轴向退开距离：

```text
axial_standoff = dot(center_to_camera, ideal_camera_direction)
```

横向偏移：

```text
lateral_offset = center_to_camera - axial_standoff * ideal_camera_direction
lateral_error = ||lateral_offset||
```

参数：

```yaml
lateral_max_m: 0.04
weight_lateral: 6.0
```

含义：

- `lateral_max_m`：硬门槛，允许相机偏离理想观察轴线的最大横向距离。
- `weight_lateral`：软代价权重，越大越倾向贴近管轴外侧观察线。

调参判断：

- `lateral_error` 很小但画面仍裁切，说明问题不是横向偏移，而是 FOV/距离/目标尺度。
- `lateral_error` 经常擦边，增大 `weight_lateral`。
- 经常因横向约束无法发布，适当放大 `lateral_max_m`。

## 5. 新增 FOV 投影约束

### 5.1 为什么 FOV 可以执行前预测

虽然真实画面要机械臂执行后才能看到，但候选 `q` 的理论成像可以执行前预测：

```text
q -> FK(q) -> T_base_cam(q)
```

再结合目标几何：

```text
C = 管口中心
a = 管轴方向
r = rim_radius_m
```

即可构造 rim 采样点并投影到图像平面。

### 5.2 相机内参来源

当前系统并不是标准 `realsense2_camera` 节点在 ROS 图中发布 `/camera/.../camera_info`，检测节点直接用 `pyrealsense2` 启动 D435i：

```python
rs_cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
self.intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
```

本轮新增：

```text
cylinder_detection -> /vision/camera_info
view_goal_solver   <- /vision/camera_info
```

检测节点把当前 D435i color stream 的真实读数发布为 `sensor_msgs/CameraInfo`：

```text
width, height, fx, fy, cx, cy
```

确认方式：

```bash
ros2 topic echo /vision/camera_info --once --full-length
```

### 5.3 FOV 约束计算

采样点：

```text
C                 管口中心
B = C + depth*a   管内代理点
rim_samples       管口圆周采样点，默认 16 个
```

投影：

```text
X_cam = inverse(T_base_cam(q)) * X_base
u = fx * X_cam.x / X_cam.z + cx
v = fy * X_cam.y / X_cam.z + cy
```

有效条件：

```text
X_cam.z > 0
image_margin_px <= u <= width  - image_margin_px
image_margin_px <= v <= height - image_margin_px
```

评分：

```text
fov_score = inside_count / total_count
fov_penalty = 1 - fov_score
```

当前参数：

```yaml
enable_fov_constraint: true
rim_radius_m: 0.025
rim_sample_count: 16
image_margin_px: 30.0
min_fov_score: 0.95
weight_fov: 8.0
```

调试字段：

```text
fov_score
fov_inside_count
fov_total_count
projected_center_uv
projected_min_uv
projected_max_uv
```

解释：

- `projected_center_uv`：预测管口中心在图像中的位置。
- `projected_min_uv` / `projected_max_uv`：所有采样点投影后的像素包围范围。
- 如果圆口被裁，通常 min/max 会越过 `image_margin_px` 留白边界。

注意：

`min_fov_score: 0.95` 在 `rim_sample_count: 16` 时总点数为 18，因此基本等价于要求所有采样点都在带 margin 的图像区域内，因为 `17/18 = 0.944 < 0.95`。

## 6. 当前调参现象解释

### 6.1 `gaze_max_rad` 调大后求解值也跟着变大

现象：

```text
gaze_max=0.23 -> 求解大多 0.24+
gaze_max=0.25 -> 求解大多 0.26+
```

原因：

`gaze_max_rad` 是超限惩罚边界。阈值越宽，优化器越可能选择一个 gaze 稍差但 motion、wrist、standoff 或其他项更好的折中解。

此外超限惩罚使用平方项：

```text
(gaze_error - gaze_max_rad)^2
```

如果只超 `0.01 rad`，即使 `hard_constraint_weight=200`，惩罚也只有：

```text
200 * 0.01^2 = 0.02
```

可能压不过其他 cost 的收益。

若希望 gaze 更严格：

- 提高 `weight_gaze`，让 gaze 在整个优化区域持续重要。
- 提高 `hard_constraint_weight`，让超限更痛。
- 更根本地实现 `best_feasible` 选择：同时记录总分最优解和满足硬约束的最优解，优先发布 feasible 解。

### 6.2 顶部目标表现好，侧面目标 rim 不完整

顶部目标满足需求，不代表侧面目标也会自然满足。侧面目标更容易达到完美几何对准：

```text
gaze_error 很小
axis_error 很小
lateral_error 很小
distance_to_center 接近 standoff_desired_m
```

但如果没有 FOV 约束，优化器不知道 rim 是否完整落在画面内，可能选出“几何几乎完美，但圆口被图像边界裁掉”的姿态。

本轮 FOV 约束正是为了解决这个问题：让候选姿态在执行前预测 rim 是否完整位于图像内。

## 7. 调参建议

### 7.1 先分清失败来源

查看完整 debug：

```bash
ros2 topic echo /view_goal_solver_debug --field data --full-length --once
```

优先判断：

```text
constraints_satisfied
published
gaze_error / axis_error
distance_to_center / standoff_error
lateral_error / axial_standoff
fov_score / projected_min_uv / projected_max_uv
joint_limit_margin
```

若失败：

- `gaze_error` 超限：优化方向没看准管内代理点。
- `axis_error` 超限：光轴和管轴不够平行。
- `lateral_error` 超限：相机站位偏离管轴外侧观察线。
- `fov_score` 超限：预测图像中 rim/中心/代理点不完整或太贴边。
- `joint_limit_margin` 接近 0：姿态靠近关节限位。

### 7.2 约束与权重不要混淆

硬阈值：

```yaml
gaze_max_rad
axis_max_rad
lateral_max_m
standoff_min_m
standoff_max_m
min_fov_score
```

权重：

```yaml
weight_gaze
weight_axis
weight_standoff
weight_lateral
weight_fov
weight_motion
weight_joint_limit
weight_wrist_motion
hard_constraint_weight
```

经验：

- 降低阈值是“更严格拒绝差解”。
- 提高权重是“优化时更偏好好解”。
- 若 best 总分最低但不满足阈值，当前 solver 会拒绝发布；并不会自动找第二好 feasible 解。

### 7.3 距离和 FOV 的关系

若画面中圆口太满或被裁：

优先看：

```text
fov_score
projected_min_uv
projected_max_uv
distance_to_center
axial_standoff
```

可调：

```yaml
standoff_desired_m: 0.12
standoff_min_m: 0.09
standoff_max_m: 0.18
image_margin_px: 40.0
min_fov_score: 0.95
weight_fov: 12.0
```

若 FOV 太严格导致频繁不发布：

```yaml
image_margin_px: 20.0
min_fov_score: 0.90
weight_fov: 6.0
```

### 7.4 侧面目标与顶部目标可考虑分场景参数

侧面目标：

- 更容易几何对准，但更容易因近距离、矩形画幅、roll 或遮挡造成 rim 不完整。
- 建议更重视 FOV、standoff、图像 margin。

顶部/近竖直目标：

- 可达性和腕部姿态更敏感。
- 过强的 axis/gaze/lateral 约束可能导致不可发布。
- 需要关注 `joint_limit_margin`、`wrist_motion_cost`、plan success。

后续可根据 `axis_raw_base` 与 base Z 的夹角或 `observability_state` 分组使用不同参数。

## 8. 后续建议

### 8.0 5-5 追加：standoff 改为轴向距离约束

调试发现 `distance_to_center` 会把横向偏移也算进距离，导致候选姿态可能“欧氏距离够远”，但相机实际没有站到圆柱轴线上方。已将 solver 的 `standoff_error`、`standoff_min_m/max_m` 硬约束改为使用：

```text
axial_standoff = dot(camera_position - center, -view_axis)
standoff_error = axial_standoff - standoff_desired_m
```

`distance_to_center` 仍保留在 debug 中用于观察实际三维距离，但不再作为 standoff 约束依据。debug 额外输出 `standoff_reference:"axial_standoff"` 和 `constraint_failures`，用于直接查看失败约束及差值。

### 8.1 已实现 best feasible

当前 `optimize()` 已改为同时维护：

```text
best_overall
best_feasible
```

发布优先级：

```text
if best_feasible exists:
  publish best_feasible
else:
  publish debug of best_overall and reject
```

这样 `axis_max_rad`、`lateral_max_m`、`standoff_min_m/max_m` 等才更像真正的硬约束，而不是“总分最低解最后检查失败就整轮失败”。debug 新增：

```text
selected_candidate
has_feasible_candidate
best_overall_score
best_feasible_score
best_overall_constraint_failures
```

如果 `selected_candidate=best_feasible`，说明虽然可能存在分数更低但不可行的点，最终发布的是满足约束的最优点。如果 `has_feasible_candidate=false`，debug 展示的是 `best_overall`，用于继续排查不可行原因。

### 8.2 加入真实 rim 半径来源

当前 FOV 使用：

```yaml
rim_radius_m: 0.025
```

它来自 `REAL_TUBE_DIAMETER=0.050` 的半径。若后续检测多个规格管口，应将半径作为语义字段发布，并由 solver 优先使用消息里的半径。

### 8.3 执行后闭环验证

FOV 是执行前预测，仍会受标定、视觉中心误差、半径误差影响。执行成功后仍应记录：

```text
actual_camera_position
actual_camera_z
actual gaze_error
actual axis_error
actual image rim completeness
```

长期应把执行后真实图像验证结果和 solver debug 对齐，判断 FOV 预测误差来源。
