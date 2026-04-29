# 2026-04-29 Current Analysis: Remaining Possible Issues

本文只记录当前仍值得排查的问题。已经由实物、TF 可视化或本轮代码修改确认排除的点，不再纳入可能原因。

## 1. 已确认不再作为问题项

- 手眼标定整体方向不作为当前首要怀疑点：TF 可视化中 `camera_link` 朝向与实物安装一致。
- `tcp_link`、`gripper_link`、`link6` 在 TF 图中完全重合，不把末端 link 偏置作为当前解释。
- 之前的 calib bug 已修复，不再回到旧 bug 假设。
- side-line 不参与管口圆/椭圆判定，也不混入 rim candidate 质量评分；当前只用于单圆双解几何消歧。
- `[SELECTED]` 诊断输出已经限定为当前选中目标，不再输出所有识别圆柱。
- 当前主问题不再是 P0/P1 双解频繁翻转。新日志中 `P=1`、`side.applied=True`、`side.score` 接近 1、`margin` 接近 1，说明侧边几何消歧本身在选中目标上是稳定生效的。
- `/vision/cylinder_pose` 与 `/cylinder_pose_base` 中的 orientation 是占位值，这不是当前 bug。真正传给控制层的是语义消息中的 `top_center_*` 和 `axis_*`。

## 2. 当前最可能的问题

### 2.1 目标生成策略过于轴向化，导致可达性差

控制层当前策略在 `src/vision_arm_control/vision_arm_control/arm_pose_controller.py` 中：

```text
view_axis = _resolve_view_axis(center, axis_raw)
camera_position = center - view_axis * camera_standoff_m
look_at_point = center
```

也就是：相机沿圆柱轴线外侧退开固定距离，并让相机光轴直接看向管口中心。

对 ID=366 的日志：

```text
center(base)    = [0.3902, -0.0053, 0.1668]
axis/view_axis  = [-0.1280, 0.0306, -0.9913]
camera(base)    = [0.4030, -0.0084, 0.2659]
tool(base)      = [0.4003, 0.0345, 0.3235]
```

这表示目标圆柱在机器人前方约 39 cm、左右接近中线、离 base 高约 16.7 cm；轴向几乎沿 base 的负 Z，即近似竖直朝下。控制层因此要求相机站到管口中心上方约 10 cm 的位置做近似俯视。

从实物照片看，相机是侧装在末端上的。对这种安装，要求相机中心在目标管口正上方且光轴近乎垂直向下，容易带来：

- 腕部姿态很极端
- 相机/末端夹具离目标和木架太近
- IK 可行域窄
- MoveIt 多次规划失败

因此，对 ID=366 这种竖直管口，重复规划失败更像是目标位姿策略/可达性问题，而不是视觉侧边消歧问题。

### 2.2 `camera_standoff_m=0.10` 可能过近

当前参数在 `src/vision_arm_control/config/arm_pose_controller_bottom_scan.params.yaml`：

```yaml
camera_standoff_m: 0.10
```

10 cm 是相机中心到管口中心的距离，不是相机外壳到目标的安全距离。侧装相机、线缆、末端结构都会吃掉这段空间。对竖直管口，这个距离可能使末端目标落入碰撞或不可达区域。

后续测试建议优先尝试更保守的距离，例如 0.16-0.22 m，或者做可达性驱动的多候选站位，而不是只发布唯一的轴向站位。

### 2.3 近竖直轴向下的 roll 选择可能让腕部姿态困难

当前 `roll_reference_axis` 是 `world_z`。当 `view_axis` 接近 `+Z/-Z` 时，用 world_z 构造相机 x/y 轴会变得数值上不太稳定，且可能选出对机械臂不友好的腕部 roll。

这通常不一定造成“光轴不指向目标”，但会明显影响 IK/MoveIt 成功率。后续更合理的策略是：

- 为同一个 camera position 生成多个 roll 候选
- 或使用当前相机/腕部姿态作为 roll 连续性参考
- 或在接近竖直轴时改用固定的、经实机验证更可达的 roll reference

### 2.4 MoveIt orientation tolerance 已不是早期极宽值，但仍需实测闭环误差

当前 `src/control/config/vision_moveit_executor.params.yaml` 中：

```yaml
goal_orientation_tolerance: 0.15
```

0.15 rad 约等于 8.6 度，已经不是早期 0.6 rad 那种会显著放宽朝向的设置。它可能导致成功规划后仍有可见的中等角度偏差，但无法单独解释“多次规划失败”。

若后续出现“MoveIt 成功但相机光轴仍明显偏离圆柱轴向”，应当直接在执行后计算：

```text
actual_camera_axis = base->camera_color_optical_frame 的 +Z
desired_to_center  = center_base - actual_camera_position
angle_error        = acos(dot(actual_camera_axis, desired_to_center))
```

不要只凭外部照片判断，因为照片不容易区分真实光轴、相机外壳朝向和视觉覆盖区域。

### 2.5 可视化紫色 target 不是控制层真实目标

视觉窗口里的紫色 `target` 来自 `detection_node.py::draw_target_pose_info()`，是旧的局部可视化逻辑：

```text
p_cam_des = c_3d - camera_view_dir * scan_distance
```

它使用视觉节点自己的 `scan_distance=0.05` 和 `look_inward`，并不是 `/target_pose_stamped` 中 MoveIt 接收的真实 tool pose。控制层真实站位使用 `camera_standoff_m=0.10`，且还会经过 `camera_color_optical_frame -> camera_link -> tool` 的标定链。

因此，图像中紫色 target 很低，首先说明可视化提示容易误导；它不是单独证明真实 MoveIt 目标也在同一像素位置。后续应把这个 overlay 改成与 `/target_pose_debug` 或控制层策略一致，或者明确标注为视觉本地预测。

### 2.6 单次 first-frame 策略会放大偶发观测误差

控制层当前是 single-shot：

- 选中目标后只取第一帧 `/cylinder_semantics_base`
- 发布一次 `/target_pose_stamped`
- 后续视觉更新全部忽略

这个策略有利于排除时序追踪干扰，但会把第一帧的偶发中心/轴向误差直接锁定为目标位姿。对当前 ID=366 日志，侧边消歧稳定，主问题更像可达性；但后续若同一目标偶尔生成明显离谱姿态，需要考虑先采若干帧做一致性确认，而不是立即使用第一帧。

### 2.7 side-line 尚未修正圆心/轴线，只完成双解消歧

当前 side-line 只决定 P0/P1，不修正 rim center，也不基于两条母线反推出更完整的圆柱轴线。如果图像中圆口遮挡、rim 拟合偏心，或者单圆几何对中心/轴向仍有残差，控制层收到的 `top_center` 与 `axis` 仍可能有系统偏差。

这不是当前 ID=366 MoveIt 失败的首要解释，但仍是视觉几何后续精化方向。

## 3. 建议的下一步验证顺序

1. 保持当前视觉 side-line 消歧不动，先验证控制目标是否可达。
2. 对 ID=366 这类近竖直管口，尝试把 `camera_standoff_m` 提高到 0.16-0.22 m，并记录 MoveIt 成功率。
3. 如果仍失败，引入多个 roll 候选或斜向观察候选，让控制层选择 IK/MoveIt 可行的位姿。
4. 若规划成功但画面仍偏轴，执行后用 TF 计算实际 `camera_color_optical_frame +Z` 与目标中心方向的夹角。
5. 将视觉窗口中的紫色 target overlay 与控制层 `/target_pose_debug` 对齐，避免继续用旧可视化判断真实目标位姿。
