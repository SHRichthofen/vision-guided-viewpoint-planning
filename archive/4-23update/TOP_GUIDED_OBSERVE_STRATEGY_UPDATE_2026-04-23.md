# 4-23 更新记录：候选观察位姿改为 Top-Guided 策略

日期：2026-04-23

## 背景

现场反馈表明，当前 `bottom_ring_candidates` 的几何假设不合适：

1. 控制层把视觉给出的“管口中心 + 朝内法向”继续推成“底部 ROI”
2. 候选围绕一个推断出来的 `bottom_roi` 做 look-at
3. 这会把 `tube_length_m / axis_into_tube_sign / bottom_roi_offset_m` 的误差放大成目标位姿误差
4. 一旦轴向符号或底部方向判断不稳，候选就会跳到管子的错误一侧，甚至把机械臂往桌面压

从任务目标看，我们真正已知的只有：

- 大致的管口中心
- 一个用于“看进去”的轴向

而任务真正需要的是：

- 从管口外侧生成稳定、可执行、能看进管内的观察位姿

这并不要求显式估计“底部 ROI”。

## 本次改动

### 1. 控制层主策略改为 `top_guided_candidates`

修改文件：

- `src/vision_arm_control/vision_arm_control/arm_pose_controller.py`
- `src/vision_arm_control/config/arm_pose_controller_bottom_scan.params.yaml`

核心变化：

1. 默认 `target_mode` 从 `bottom_ring_candidates` 改为 `top_guided_candidates`
2. 候选生成不再使用：
   - `bottom_roi`
   - `candidate_radius_list_m`
   - `candidate_axial_offset_list_m`
3. 候选改为直接围绕 `top_center`（管口中心）生成
4. 相机统一从管口外侧退开 `camera_standoff_m + candidate_standoff_offset`
5. 相机再在管口法平面上做有限横向偏移 `candidate_lateral_offset`
6. 所有候选统一 look-at 到：
   - `top_center + view_axis * look_at_depth_m`

这里的 `look_at_depth_m` 只是“往管内看一点”，不再等同于“底部估计位置”。

### 2. 视线轴向改为优先根据当前相机所在侧自动判定

旧问题：

- 之前控制层依赖固定 `axis_into_tube_sign`
- 如果现场轴向正负方向与配置不一致，候选就会被翻到错误一侧

现在改为：

1. 优先查询当前 `desired_camera_frame` 在 `target_frame` 下的位置
2. 根据“当前相机在管口哪一侧”，自动选择 `view_axis` 的符号
3. 只有在当前相机 TF 查询失败时，才退回 `axis_into_tube_sign`

这样可以显著减少“目标一刷新就跳到另一侧”的问题。

### 3. 候选排序不再对管底做隐式偏置

新的评分偏好：

1. 高质量观测时优先较小横向偏移
2. 近圆退化或低质量时优先稍微斜视的候选
3. 轻微偏好更安全的较大 standoff
4. 若能获取当前相机位置，则轻微偏好与当前观察方位更接近的 azimuth

这样生成的候选更像“从当前可观察侧，围绕管口做有限重定位”，而不是“围着一个虚构底点找姿态”。

### 4. 兼容旧模式名，但行为已映射

为了不让旧 launch / yaml 立刻失效：

- 若外部仍传入 `bottom_ring_candidates`
- 控制层会给出 warning
- 但内部已统一映射到 `top_guided_candidates`

也就是说，旧名字暂时还能跑，但不再使用底部 ROI 推断逻辑。

## 新参数语义

当前参数文件改成：

- `look_at_depth_m`
- `camera_standoff_m`
- `candidate_lateral_offset_list_m`
- `candidate_standoff_offset_list_m`
- `low_quality_lateral_scale`
- `high_quality_lateral_scale`

相比旧参数，新的命名更贴近真实几何动作：

- lateral：横向偏移
- standoff：外侧退开距离
- look_at_depth：朝管内看多深

## 设计结论

这次调整的核心不是“让系统更复杂”，而是把观察位姿重新建立在任务真正可信的输入上：

- 信任：管口中心
- 信任：朝内轴向
- 不再过度信任：底部位置的显式几何估计

对于“观察圆柱底部目标”这个任务，更合理的做法是：

1. 从管口外侧获得一个可执行的观察位姿
2. 视线朝管内
3. 必要时用轻微斜视来打破近圆退化

而不是在控制层里先构造一个强假设的 `bottom_roi` 再围着它采样。

## 建议验证

1. 观察 `/target_pose_debug`
   - 确认 `best_candidate` 中已经没有 `bottom_roi / axial_offset / radius`
   - 改为 `view_axis / lateral_offset_m / standoff_m`
2. 固定同一目标时，查看候选是否保持在当前观察侧附近
3. 在垂直放置、靠近桌面的目标上，确认候选不再明显向桌面方向压低
4. 若需要再提升“看底部”的概率，优先调：
   - `look_at_depth_m`
   - `candidate_lateral_offset_list_m`
   - `candidate_standoff_offset_list_m`

## 本次未做

1. 未改执行器的可达性筛选逻辑
2. 未引入新的阶段状态机
3. 未修改视觉层法向估计方式

本轮只聚焦一个问题：

> 把控制层的候选观察位姿，从“底部假设驱动”改回“管口与轴向驱动”。
