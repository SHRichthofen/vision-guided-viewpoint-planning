# View Goal Solver 优化方程调整记录

日期：2026-05-05

## 1. 本地备份

本次修改前已备份：

```text
archive/5-5update/view_goal_solver.cpp.backup_2026-05-05_pre_objective_simplification
archive/5-5update/view_goal_solver.params.yaml.backup_2026-05-05_pre_objective_simplification
```

## 2. 调整动机

前一版 solver 同时使用了多个强约束：

```text
axial_standoff_min/max
lateral_max
fov_score_min
gaze_max
axis_max
固定 standoff_desired_m
```

这些量并非独立。对于圆柱观察位姿来说，`axial_standoff`、`lateral_error`、`gaze_error`、`axis_error` 和 FOV 投影质量都在描述相机相对圆柱轴线的位置与朝向。当前日志中常见的失败组合：

```text
axial_standoff 不够
lateral_error 偏大
fov_score 太低
gaze_error 太大
axis_error 太大
```

更像是同一个姿态偏差被多个耦合指标重复惩罚，而不是五个彼此独立的问题。对星型排布的小管阵列来说，固定 `standoff_desired_m` 也不稳定：不同管子的可达空间不同，很难定义一个对所有目标都合适的 desired standoff。

## 3. 新的优化分层

本次将目标函数调整为：

```text
硬约束：
- 轴向距离在区间内
- FOV inside ratio 达到最低要求
- 关节不越界

软目标：
- axis 对齐，作为较强 cost
- FOV 边界余量，作为连续 cost
- gaze、lateral、motion、wrist 作为弱 cost
```

`lateral_max`、`gaze_max`、`axis_max` 不再默认作为硬约束，但保留参数和可选开关：

```yaml
enforce_lateral_max: false
enforce_gaze_max: false
enforce_axis_max: false
```

需要恢复强硬门槛时，可以单独打开。

## 4. standoff 方程

旧版本：

```text
standoff_error = axial_standoff - standoff_desired_m
```

这会强迫所有目标追同一个 desired 距离。

新版本：

```text
axial_standoff = dot(camera_position - center, -view_axis)

if axial_standoff < standoff_min_m:
  standoff_error = axial_standoff - standoff_min_m
elif axial_standoff > standoff_max_m:
  standoff_error = axial_standoff - standoff_max_m
else:
  standoff_error = 0
```

也就是区间 hinge loss。区间内不惩罚，让 FOV、axis 和可达性决定实际距离。

当前建议参数：

```yaml
standoff_min_m: 0.055
standoff_max_m: 0.17
```

`standoff_desired_m` 暂时保留为兼容参数和 debug 输出，不参与当前 cost。

## 5. FOV 方程

旧版本 FOV cost：

```text
fov_penalty = 1 - inside_count / total_count
```

这是阶跃信号。投影点从边界内移动到边界外时，cost 突然跳变；但在边界内移动时没有连续反馈。因此它只会告诉 solver “有没有出画”，不会告诉 solver “离边缘还有多少余量”。

新版本仍保留：

```text
fov_score = inside_count / total_count
```

用于硬约束和 debug，但 soft cost 改为连续边界余量惩罚：

```text
edge_margin = min(u, width - u, v, height - v)
margin_shortage = image_margin_px - edge_margin

if margin_shortage > 0:
  penalty += (margin_shortage / image_margin_px)^2
```

这样投影点越贴边或越出界，惩罚越大；点完全在安全 margin 内则不产生 FOV soft penalty。

debug 新增：

```text
fov_penalty
fov_min_margin_px
```

`fov_min_margin_px` 为负数时，表示至少有一个采样点已经出画；为小正数时，表示目标虽然在画面内但贴边。

## 6. 当前目标函数

当前总分近似为：

```text
score =
  weight_gaze      * gaze_alignment
+ weight_axis      * axis_alignment
+ weight_standoff  * standoff_interval_cost
+ weight_lateral   * lateral_cost
+ weight_fov       * fov_margin_penalty
+ weight_motion    * motion_cost
+ weight_joint     * joint_limit_cost
+ weight_wrist     * wrist_motion_cost
+ hard_weight      * hard_constraint_violation
```

其中：

```text
gaze_alignment = 1 - dot(camera_z, normalize(bottom_proxy - camera_position))
axis_alignment = 1 - dot(camera_z, view_axis)
```

当前建议权重：

```yaml
weight_gaze: 4.0
weight_axis: 12.0
weight_standoff: 6.0
weight_lateral: 2.0
weight_fov: 16.0
```

含义：

- `axis` 比 `gaze` 更重要，避免为追 bottom proxy 产生过强斜视角。
- `FOV` 权重提高，但通过连续 margin penalty 给出方向性反馈。
- `lateral` 降为弱偏好，不再默认卡死星型阵列中难以正对的目标。
- `standoff` 只在区间外惩罚。

## 7. 观察重点

下一轮测试优先看：

```text
constraint_failures
axial_standoff
fov_score
fov_penalty
fov_min_margin_px
axis_error
lateral_error
published
```

若仍然频繁不发布：

- 如果主要失败是 `fov_score_min`，先看 `fov_min_margin_px`。负得很多说明目标确实出画；接近 0 说明 margin 太严。
- 如果 `axial_standoff_min` 仍失败，说明机械臂没有搜索到更高/更远位置，优先检查搜索范围与可达性。
- 如果已经发布但画面斜，增加 `weight_axis`，不要急着重新打开 `axis_max` 硬约束。
- 如果目标完整但贴边，增加 `image_margin_px` 或 `weight_fov`。

## 8. 追加：底部 rim FOV 约束

实际测试发现，仅检查：

```text
top center
bottom center proxy
top rim
```

不足以保证圆柱底面完整可见。底部中心点在画面内时，底部圆盘边缘仍可能贴边或出画；而检测任务真正需要的是底部区域有足够成像余量。

因此新增底部圆环采样：

```text
bottom_center = center + depth_proxy_m * view_axis
bottom_rim_i = bottom_center + rim_radius_m * (cos(theta_i) * u + sin(theta_i) * v)
```

其中 `u`、`v` 是垂直于 `view_axis` 的圆环平面基向量。

新的 FOV 采样集合变为：

```text
top center
bottom center
top rim samples
bottom rim samples
```

并且 bottom rim 单独统计：

```text
bottom_rim_fov_score = bottom_rim_inside_count / bottom_rim_total_count
bottom_rim_fov_penalty = average bottom rim margin shortage penalty
bottom_rim_min_margin_px = min bottom rim edge margin
```

当前参数：

```yaml
min_bottom_rim_fov_score: 0.95
weight_bottom_rim_fov: 24.0
```

由于当前 `rim_sample_count=16`，`min_bottom_rim_fov_score=0.95` 实际上要求 16 个底部圆环采样点全部在安全边界内；15/16 只等于 0.9375，会失败。这样可以避免“总 FOV 17/18 通过，但底部边缘仍不可见”的情况。

下一轮 debug 重点看：

```text
bottom_rim_fov_score
bottom_rim_min_margin_px
bottom_rim_inside_count
bottom_rim_total_count
projected_bottom_center_uv
```

若 `bottom_rim_fov_score=1.0` 但图像里仍看不到完整底面，说明问题不再是成像边界，而更可能是视角斜导致的自遮挡，需要提高 `weight_axis` 或重新启用 `enforce_axis_max`。
