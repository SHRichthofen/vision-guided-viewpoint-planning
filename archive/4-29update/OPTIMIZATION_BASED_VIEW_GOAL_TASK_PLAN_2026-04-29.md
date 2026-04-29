# 2026-04-29 Task Plan: Optimization-Based View Goal Solver

本文记录下一阶段任务计划：把“根据管口中心和轴向生成机械臂观察位姿”的问题，从当前单一 `PoseStamped` 生成，升级为以视觉约束和机械臂可达性共同驱动的优化式目标求解。

## 1. 当前结论

当前控制层问题不只是 IK solver 弱，也不只是 `camera_standoff_m` 或 MoveIt tolerance 参数不合适。

核心问题是：当前目标生成把观察任务硬编码成唯一相机姿态：

```text
camera_position = center - view_axis * camera_standoff_m
look_at_point = center
roll = 由 roll_reference_axis(world_z) 间接决定
```

这会导致：

- 相机必须站在管轴外侧固定距离
- 光轴必须严格看向管口中心
- 绕光轴 roll 不是自由优化变量，而是由 `world_z` 参考轴固定生成
- 近竖直管口时 roll 参考容易退化或产生机械臂不舒服的腕部姿态
- MoveIt 只收到一个 tool pose，然后用当前 IK 插件黑盒求解

因此，单独把 KDL 换成 pick_ik/TRAC-IK 可以提升某些 pose 的 IK 成功率，但不能从根本上解决“roll 应该作为自由度被优化选择”的问题。

根据最新判断，当前大部分 MoveIt 规划失败主要来自目标位姿在 MoveIt 内部目标点/IK 采样阶段失败。无需再把“只读评估器”作为前置阶段。后续计划直接进入：

```text
1. 更换 KDL -> pick_ik
2. 实现 view_goal_solver 优化器
3. 让优化器输出更可达的 q_goal / target pose
4. 再由 MoveIt 执行
```

## 2. 不再优先推进的方向

以下方向可以作为对照实验，但不应作为主方案：

- 只把 `camera_standoff_m` 调大或调小
- 只把 MoveIt `goal_position_tolerance` / `goal_orientation_tolerance` 放宽
- 只把 `roll_reference_axis` 从 `world_z` 换成 `world_x/world_y`
- 只增加离散观察点数量
- 只替换 IK solver，然后继续发布唯一 `/target_pose_stamped`

这些做法可能缓解个别场景，但仍然没有把视觉任务本身表达成可优化约束。

## 3. 目标架构

新增或重构一个观察目标求解层，暂称：

```text
view_goal_solver
```

它位于 `/cylinder_semantics_base` 与 MoveIt executor 之间：

```text
/vision/selected_cylinder_semantics
  -> vision_to_arm_transform
  -> /cylinder_semantics_base
  -> view_goal_solver
  -> q_goal 或 scored target pose
  -> vision_moveit_executor
```

建议最终优先输出 `q_goal`，而不是只输出 `PoseStamped`。原因是：如果前置求解层已经找到一个评分更好的 IK 解，再把 pose 交给 MoveIt，MoveIt 可能重新选择另一个不舒服的 IK 解。

## 4. 优化问题定义

输入：

```text
C = 管口中心，base frame
a = 管轴方向，base frame，约定指向管内或由当前逻辑解析 view_axis
q0 = 当前机械臂关节角
T_tool_camera = 手眼标定
camera intrinsics / FOV
可选：管口半径、管深或管底代理距离
```

决策变量优先使用关节角：

```text
q = robot joint configuration
```

通过 FK 得到：

```text
p_cam(q) = camera_color_optical_frame 在 base 下的位置
z_cam(q) = camera optical +Z 在 base 下的方向
R_cam(q) = camera 姿态
```

管内代理点：

```text
B = C + depth_proxy * a
```

若管深未知，先使用参数化代理距离，例如 `0.03-0.08 m`，后续再由检测结果或配置给出。

## 5. 目标函数

建议构造加权目标：

```text
minimize J(q) =
  w_gaze      * gaze_error(B, p_cam(q), z_cam(q))
+ w_axis      * axis_parallel_error(z_cam(q), a)
+ w_dist      * standoff_error(||C - p_cam(q)||)
+ w_fov       * fov_penalty(C, B, rim_samples, camera intrinsics)
+ w_motion    * joint_motion_cost(q, q0)
+ w_limit     * joint_limit_cost(q)
+ w_manip     * manipulability_cost(q)
+ w_roll      * roll_comfort_cost(q 或 R_cam(q))
+ w_collision * collision_margin_cost(q)
```

含义：

- `gaze_error`：管底代理点应落在相机视锥中心附近
- `axis_parallel_error`：相机光轴与管轴尽量平行或接近平行
- `standoff_error`：距离在安全且清晰的观察范围内
- `fov_penalty`：管口中心、管底代理点、rim 采样点应在 FOV 内
- `joint_motion_cost`：优先选择离当前姿态近的解
- `joint_limit_cost`：远离关节限位
- `manipulability_cost`：避开奇异或低可操作度姿态
- `roll_comfort_cost`：让 roll 由机械臂舒适度和视觉质量共同决定，而不是固定 `world_z`
- `collision_margin_cost`：保持安全距离

## 6. 硬约束

硬约束应至少包括：

```text
joint limits
self collision free
scene collision free
d_min <= ||C - p_cam(q)|| <= d_max
angle(z_cam(q), B - p_cam(q)) <= theta_gaze_max
angle(z_cam(q), a) <= theta_axis_max
```

MoveIt 的 goal tolerance 不能替代这些视觉约束。MoveIt tolerance 只应作为执行层允许误差，而不是观察质量定义。

## 7. 分阶段实施计划

### Phase 0: 保持视觉链路不动

继续保持视觉层只发布语义：

```text
top_center_cam/base
axis_cam/base
```

不要让视觉层发布伪完整四元数姿态。`/vision/cylinder_pose` 与 `/cylinder_pose_base` 的占位 orientation 仍只作为兼容接口，不进入主控制逻辑。

### Phase 1: 立即替换 pick_ik

当前 MoveIt 配置为 KDL：

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

下一步直接改为 pick_ik 路线：

```yaml
arm:
  kinematics_solver: pick_ik/PickIkPlugin
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3
  mode: global
  position_scale: 1.0
  rotation_scale: 0.5
  position_threshold: 0.001
  orientation_threshold: 0.02
  cost_threshold: 0.001
```

说明：

- pick_ik 不是最终完整方案，但应立即替换 KDL，降低目标点采样/IK 失败概率。
- `kinematics_solver_timeout` 从 `0.005s` 提高到 `0.05s` 量级，避免过早失败。
- 若 `pick_ik` 安装或兼容性受阻，再使用 TRAC-IK 作为备选。
- 不再等待额外评估阶段再决定是否替换。

### Phase 2: 直接实现 view_goal_solver 优化器

新增或重构 `view_goal_solver`，主输入仍是：

```text
/cylinder_semantics_base
```

优化器直接求解观察目标，不再先做只读评估器。调试字段可以伴随实现输出，但不作为前置阶段。

第一版优化器必须以关节角 `q` 为决策变量，直接在 `q -> FK(q) -> camera pose -> cost/constraint` 这条链路上求解。

严禁把本任务实现成“离散 pose 猜测 + 打分 + MoveIt plan 过滤”的伪优化器。以下做法明确禁止作为主方案或临时主链路：

1. 禁止枚举若干 `standoff / tilt / roll / lateral_offset` 组合生成一批完整 `PoseStamped`。
2. 禁止对这些离散 pose 候选逐个调用 `setPoseTarget()` / `plan()`，再从成功轨迹末端提取 `q_goal`。
3. 禁止使用缺乏机器人学或视觉约束来源的离散采样评分，把失败交给 MoveIt 黑盒反复试错。
4. 禁止用“多候选 pose 规划成功率”替代真正的 `q` 空间优化。
5. 禁止在 solver 内部制造大量 MoveIt pose-goal 规划请求来猜测可达姿态。

允许的多初值只用于连续优化或明确的 IK 求解器初始化，不允许退化成离散候选猜谜。每一个被比较的候选都必须是明确的关节解 `q`，并且所有视觉误差都必须由 FK 后的真实 `camera_color_optical_frame` 位姿计算。

核心要求：roll 不再由 `world_z` 单独决定，也不能靠离散 roll 枚举猜测；它必须作为 `q` 空间优化结果，通过关节限位、运动代价、视觉约束和必要的 IK/优化约束共同决定。

### Phase 3: executor 支持 joint target 规划

在当前 MoveIt 框架内做最小架构升级：

1. `view_goal_solver` 输出最优 `q_goal`，同时保留 debug 中的相机目标和得分。
2. `vision_moveit_executor` 增加 joint target 输入路径。
3. 优先执行：

```cpp
move_group_->setJointValueTarget(q_goal)
```

而不是只执行：

```cpp
move_group_->setPoseTarget(target_pose)
```

原因：如果优化器已经选出更舒适的 IK 解，再发布 `PoseStamped` 会让 MoveIt 重新求 IK，可能丢失优化器选中的 roll/关节配置。

### Phase 4: 优化器与 MoveIt 执行联调

联调目标：

- 对每次 `/cylinder_semantics_base` 直接生成并执行一个优化后的目标。
- 如果 `q_goal` 规划失败，优化器换下一个候选或重新求解。
- 失败后不要退回旧的单一轴向 pose 作为默认路径，避免掩盖问题。
- 保留旧 `/target_pose_stamped` 路径作为 debug/对照，但主执行路径转向 joint target。

### Phase 5: 后续增强为连续优化

第一版也必须避免离散 pose 采样猜谜。若短期无法接入完整优化库，应实现最小的 `q` 空间连续优化或调用支持代价函数/多解返回的 IK 能力，并始终保持目标形式为：

```text
q* = argmin J(q)
```

可选实现路径：

- 在项目内实现轻量版 `q` 空间数值优化，FK 后计算 gaze/axis/standoff/joint cost
- 接入 pick_ik custom cost / approximate IK 能力，让求解器直接输出可评分的关节解
- 长期评估 Tesseract/TrajOpt、Drake gaze constraint、cuRobo

再次强调：禁止把“枚举若干观察 pose，再让 MoveIt 反复 `setPoseTarget()` 试错”包装成优化器。

短期不建议直接大规模迁移到 Drake/Tesseract，除非 MoveIt 内部增强路线失败。

### Phase 6: 执行后闭环验证

每次执行成功后，用 TF 和图像做真实验证：

```text
actual_camera_position = base -> camera_color_optical_frame translation
actual_camera_z        = base -> camera_color_optical_frame +Z
angle_to_bottom        = angle(actual_camera_z, B - actual_camera_position)
angle_to_axis          = angle(actual_camera_z, a)
distance_to_center     = ||C - actual_camera_position||
```

同时检查图像：

- 管口中心是否在画面内
- rim 覆盖率是否足够
- 管底/管内代理区域是否可见

这些指标应写入 `/target_pose_debug` 或新的 `/view_goal_solver_debug`。

## 8. 推荐的最小可行路线

为了控制修改代价，建议按下面顺序推进：

1. 直接更换 KDL 为 pick_ik，并提高 IK timeout。
2. 实现 `view_goal_solver`，直接生成优化后的观察目标。
3. 将 roll 从固定 `world_z` 参考改为优化变量。
4. 引入多 seed IK 解筛选，并输出 `q_goal`。
5. executor 增加 joint target 执行路径。
6. 若 `q_goal` 规划失败，优化器尝试下一个候选/seed。
7. 如果仍不稳定，再升级为更完整的连续优化式 view goal solver。

这个顺序跳过繁琐的只读评估阶段，直接把系统从 “pose-goal planning” 推向 “gaze-constrained manipulation”。

## 9. 成功标准

至少记录以下指标：

- 对同一目标重复选择时，IK 成功率
- MoveIt plan 成功率
- 平均规划时间
- 选中解的关节限位余量
- 是否出现腕部极端姿态
- 执行后相机光轴与管轴夹角
- 执行后管口/管底可见性
- 与旧单一轴向策略的对比

只有当 “规划成功” 与 “视觉可见性成功” 同时成立，才算该目标成功。

## 10. 下一步任务

下一位 agent 可以从以下任务开始：

1. 不改视觉侧 side-line 消歧。
2. 保持 `/cylinder_semantics_base` 为主输入。
3. 直接把 MoveIt kinematics plugin 从 KDL 切换到 pick_ik。
4. 实现或新增 `view_goal_solver`，必须以 `q` 为决策变量，直接输出优化后的 `q_goal`。
5. executor 增加 joint target 输入和 `setJointValueTarget(q_goal)` 执行路径。
6. `view_goal_solver` 至少输出以下 debug 字段：
   - gaze_error
   - axis_error
   - standoff_error
   - fov_score
   - ik_success
   - plan_success
   - joint_limit_margin
   - motion_cost
   - selected_reason
7. 不再把只读评估器作为前置任务；debug 只作为实现附带输出。
8. 严禁实现“离散采样 pose 候选 + MoveIt pose plan 过滤 + 轨迹末端取 q”的猜谜式 solver。

一句话目标：

```text
把 roll、standoff、偏轴观察与 IK 解选择，从固定规则升级为任务约束下的优化结果。
```

## 11. Error / Cost 项定义与获取方式

本节作为下一位 agent 实现 `view_goal_solver` 的参考。所有 error 都应围绕候选关节解 `q` 计算，而不是只围绕一个预先固定的 `PoseStamped` 计算。

基本符号：

```text
C = 管口中心，base frame
a = 管轴方向，单位向量，base frame
q = 候选关节角
q0 = 当前关节角
p = camera_color_optical_frame 位置，由 FK(q) 得到
z = camera optical +Z 方向，由 FK(q) 得到
R = camera optical frame 姿态，由 FK(q) 得到
B = C + depth_proxy * a，管底/管内代理点
T_base_cam(q) = 候选 q 下 base -> camera_color_optical_frame
```

### 11.1 `gaze_error`

含义：

```text
相机光轴是否真正看向管底/管内代理点。
```

计算：

```text
u = normalize(B - p)
gaze_error_rad = acos(clamp(dot(z, u), -1, 1))
```

也可以使用更平滑的优化形式：

```text
gaze_error = 1 - dot(z, u)
```

获取方式：

- `B` 来自 `C + depth_proxy * a`
- `p` 和 `z` 来自候选 `q` 的 FK
- `depth_proxy` 初始可设为 `0.03-0.08 m`，后续可由配置或视觉估计给出

这是最关键的视觉任务 error，因为它直接对应“能否看进管底”。

### 11.2 `axis_error`

含义：

```text
相机光轴与管轴是否近似平行。
```

若 `a` 已明确指向管内，希望：

```text
z ≈ a
```

计算：

```text
axis_error_rad = acos(clamp(dot(z, a), -1, 1))
```

若轴向正负仍可能不稳定，可以先使用无符号平行版本：

```text
axis_error_rad = min(angle(z, a), angle(z, -a))
```

但长期建议明确管内方向，使用有符号版本。

获取方式：

- `a` 来自 `/cylinder_semantics_base.axis_base`
- `z` 来自候选 `q` 的 FK

### 11.3 `standoff_error`

含义：

```text
相机距离管口不能太近，也不能太远。
```

计算：

```text
d = ||C - p||
```

硬约束：

```text
d_min <= d <= d_max
```

软惩罚：

```text
standoff_error = max(0, d_min - d)^2 + max(0, d - d_max)^2
```

如果希望接近一个理想距离：

```text
standoff_error = (d - d_des)^2
```

获取方式：

- `C` 来自 `/cylinder_semantics_base.top_center_base`
- `p` 来自候选 `q` 的 FK
- `d_min/d_max/d_des` 由配置给出

### 11.4 `fov_score` / `fov_penalty`

含义：

```text
管口中心、管底代理点、管口 rim 采样点是否落在相机视野内。
```

对任意 3D 点 `X_base`，先转到相机坐标系：

```text
X_cam = inverse(T_base_cam(q)) * X_base
```

再用相机内参投影：

```text
u = fx * X_cam.x / X_cam.z + cx
v = fy * X_cam.y / X_cam.z + cy
```

视野内条件：

```text
X_cam.z > 0
0 <= u < image_width
0 <= v < image_height
```

建议采样点：

```text
C           = 管口中心
B           = 管底/管内代理点
rim_samples = 管口圆周上的 8-16 个点
```

评分：

```text
fov_score = in_fov_points / total_points
fov_penalty = 1 - fov_score
```

可以进一步加入图像边缘 margin，避免目标贴边：

```text
edge_penalty = 对靠近图像边界的投影点加惩罚
```

获取方式：

- 相机内参来自 RealSense intrinsics
- `T_base_cam(q)` 来自 FK
- `C/B` 来自视觉语义和代理管深
- `rim_samples` 需要管口半径与管口平面两个正交基向量
- 若第一版没有可靠半径，可先只使用 `C` 和 `B`

### 11.5 `joint_motion_cost`

含义：

```text
候选解离当前机械臂姿态有多远。
```

计算：

```text
joint_motion_cost = Σ_i w_i * wrap_angle(q_i - q0_i)^2
```

获取方式：

- `q0` 来自 `/joint_states` 或 MoveIt current state
- `q` 来自候选 IK/优化解
- `w_i` 可按关节重要性配置，初始可全部为 1

该项用于偏好更短、更平滑、更容易规划的运动。

### 11.6 `joint_limit_cost` / `joint_limit_margin`

含义：

```text
候选解是否靠近关节限位。
```

对每个关节：

```text
margin_i = min(q_i - lower_i, upper_i - q_i)
```

整体余量：

```text
joint_limit_margin = min_i margin_i
```

软惩罚：

```text
joint_limit_cost = Σ_i max(0, margin_threshold - margin_i)^2
```

也可以使用更激进形式：

```text
joint_limit_cost = Σ_i 1 / (margin_i + eps)^2
```

获取方式：

- 关节上下限来自 URDF / MoveIt RobotModel
- `q` 来自候选解

该项对侧装相机任务很重要，因为许多看似可行的相机姿态会把腕部推到极限附近。

### 11.7 `manipulability_cost`

含义：

```text
候选姿态是否接近奇异位形。
```

常见指标：

```text
J = geometric Jacobian(q)
manipulability = sqrt(det(J * J^T))
manipulability_cost = 1 / (manipulability + eps)
```

或使用 Jacobian 最小奇异值：

```text
sigma_min = min(svd(J))
manipulability_cost = 1 / (sigma_min + eps)
```

获取方式：

- MoveIt RobotState 可计算 Jacobian
- 若第一版实现复杂，可先跳过该项，先用 `joint_limit_cost + joint_motion_cost` 近似约束机械臂舒适度

### 11.8 `roll_comfort_cost`

含义：

```text
相机绕 optical axis 的 roll 是否让机械臂腕部姿态舒服。
```

这是替代当前固定 `roll_reference_axis=world_z` 的关键项。

第一版建议不要直接优化相机几何 roll，而是从关节角上惩罚腕部极端姿态：

```text
roll_comfort_cost =
  w4 * normalized_distance_to_center(joint4)^2
+ w5 * normalized_distance_to_center(joint5)^2
+ w6 * normalized_distance_to_center(joint6)^2
```

或惩罚相对当前腕部姿态的大变化：

```text
roll_comfort_cost =
  (q4 - q0_4)^2 + (q5 - q0_5)^2 + (q6 - q0_6)^2
```

工程实现上也可以：

```text
对同一个 gaze/standoff 采样多个 roll
-> 分别求 IK
-> 哪个 IK 解远离限位、运动更短、manipulability 更好，就选哪个
```

获取方式：

- `q` 和 `q0` 来自候选解与当前状态
- 关节中心/限位来自 RobotModel
- 若机械臂关节命名不同，应根据实际 SRDF/URDF 确认腕部关节索引

### 11.9 `collision_margin_cost`

含义：

```text
候选解是否接近自碰撞、环境碰撞、相机/夹具撞管口或木架。
```

硬约束：

```text
collision_free(q) == true
```

软惩罚：

```text
collision_margin_cost = max(0, safety_margin - min_distance)^2
```

获取方式：

- MoveIt PlanningScene 可做碰撞检查
- 若要拿最小距离，需要使用 distance request / collision distance 查询
- 第一版可以先只做硬碰撞过滤，不做连续距离 cost

### 11.10 `plan_success` / `plan_cost`

含义：

```text
候选 q_goal 能否从当前 q0 规划过去，轨迹是否短且稳定。
```

计算：

```text
plan_success = MoveIt plan(q0 -> q_goal) 是否成功
plan_cost = trajectory_duration 或 joint_path_length
```

获取方式：

- `view_goal_solver` 或 executor 调用 MoveIt plan
- 若 plan 失败，候选直接淘汰或赋予极大 cost
- 若 plan 成功，可记录轨迹时长、关节路径长度、轨迹点数作为 tie-breaker

该项是在线选择最终目标时非常重要的实际可执行性过滤。

## 12. 第一版实现建议

第一版不必一次实现所有 error。建议最小可用集合为：

```text
gaze_error
axis_error
standoff_error
joint_motion_cost
joint_limit_margin / joint_limit_cost
plan_success
```

之后再逐步加入：

```text
fov_score
collision_margin_cost
manipulability_cost
```

实现原则：

- 每个候选都应对应一个 `q`。
- 每个 `q` 都通过 FK 得到 `p_cam/z_cam/R_cam`。
- 所有视觉 error 都基于 FK 后的真实相机位姿计算。
- roll 不再由 `world_z` 固定生成，而是由候选 `q` 的总分自然选择。
- MoveIt success 只代表规划成功，不代表观察成功；必须同时记录视觉 error。
