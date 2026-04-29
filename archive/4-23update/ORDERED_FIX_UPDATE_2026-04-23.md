# 顺序修复记录（2026-04-23）

## 背景

在阅读已有记录 [PHASE_A_UPDATE_2026-04-23.md](/home/arnoyin/agx_arm_ws/archive/4-23update/PHASE_A_UPDATE_2026-04-23.md) 后，对当前工作区实现重新核对，发现两个更基础的问题还没有真正落地修复：

1. 视觉层虽然声明了 `freeze_full_pose_on_low_quality`，但实际代码只冻结了法向，没有冻结完整 pose。
2. 控制层仍然缺少真正的发布门控和 `/vision_exec_status` busy gate，因此会持续重发目标。

本次按“先稳定输入，再限制输出”的顺序，只落地这两部分根因修复。

---

## 本次实际修改文件

1. `src/vision_detection/vision_detection/detection_node.py`
2. `src/vision_detection/config/cylinder_detection_refactor.params.yaml`
3. `src/vision_detection/vision_detection/config.py`
4. `src/vision_arm_control/vision_arm_control/arm_pose_controller.py`
5. `src/vision_arm_control/config/arm_pose_controller_bottom_scan.params.yaml`

---

## 修改 1：视觉层真正启用“完整 pose 冻结”

### 修改内容

在 `detection_node.py::_apply_quality_policy()` 中：

1. 当 `quality_score < quality_min_full_pose` 且存在历史高质量姿态时：
   - 若 `freeze_full_pose_on_low_quality = true`，则冻结完整 pose：
     - `center_3d`
     - `normal`
     - `ellipse_2d`
   - 但保留当前帧的质量与深度诊断字段，供下游判断“当前观测是否可信”
2. 若未启用完整冻结，则仍保留旧的 `normal_only` 冻结行为
3. 额外增加 `pose_freeze_mode` 诊断字段，并写入语义消息

### 为什么改

已有 4-22/4-23 文档都把“低质量时冻结完整 pose”当作关键假设，但当前真实代码只冻结了法向。这样会产生：

- 新位置 + 旧方向
- 新深度 + 旧几何中心

对控制层来说，这是一种几何不自洽输入，会让候选观察位姿生成看起来像“筛选失效”。

### 结果

现在视觉层输出变成：

- 高质量：正常更新完整 pose
- 中低质量：优先复用最后一次高质量完整 pose
- 极低质量：若 `enforce_quality_gate = true` 且低于 `quality_min_publish`，直接不发布

---

## 修改 2：视觉层默认启用质量门控

### 修改内容

在 `cylinder_detection_refactor.params.yaml` 中将：

- `enforce_quality_gate: false`

改为：

- `enforce_quality_gate: true`

### 为什么改

之前即使质量分数很低，视觉层仍会继续向下游发布“已经退化”的结果；这会让控制层和执行层在坏输入上重复工作。

### 结果

当前配置下，质量过低的观测会在视觉层就被截断，不再一路传到 `/cylinder_pose_base` 和 `/target_pose`。

---

## 修改 3：控制层增加发布门控

### 修改内容

在 `arm_pose_controller.py` 中新增参数与状态：

- `min_publish_interval_sec`
- `min_position_delta_m`
- `min_orientation_delta_rad`
- `min_score_improvement`

新增逻辑：

- `_should_publish_target()`
- `_record_published_target()`
- `_request_target_generation()`
- 四元数角距离计算与时间戳记录

### 行为变化

控制层不再“每来一帧就发一次目标”，而是变成：

1. 先生成当前最佳候选
2. 将其与上次已发布目标比较
3. 只有在以下条件满足时才发布：
   - 强制发布事件
   - 与上次目标的位置差超过阈值
   - 与上次目标的姿态差超过阈值
   - 候选分数改善明显
4. 且连续发布之间至少满足最小时间间隔

### 为什么改

这一步是为了解决“上游连续更新被直接翻译成高频 `/target_pose` 刷新”的问题。

---

## 修改 4：控制层接入 `/vision_exec_status` busy gate

### 修改内容

在 `arm_pose_controller.py` 中新增：

- `exec_status_topic`
- `busy_status_states`
- `/vision_exec_status` 订阅
- `on_executor_status()`
- `_executor_is_busy()`

### 行为变化

当执行器状态为以下任一值时：

- `planning`
- `executing`
- `queued`

控制层不会立即发布新目标，而是只记录“有新请求待处理”。

执行器恢复到非忙碌状态后，再用最新的视觉结果重新判断是否发布。

### 为什么改

执行器本身已经有 latest-wins 队列，但如果控制层完全不感知下游忙闲，上游还是会持续刷屏。这个 busy gate 是把节流责任拉回控制层。

---

## 修改 5：修正目标切换时的旧姿态误发

### 修改内容

在 `on_target_id_callback()` 中：

1. 切换目标时清空 `latest_cylinder_pose`
2. 标记一次待处理的强制发布请求
3. 不再立刻用上一目标残留的 pose 生成新目标

### 为什么改

之前目标 ID 一切换，控制层会立刻拿旧的 `latest_cylinder_pose` 生成一次目标，存在“旧目标位姿 + 新目标 ID”的瞬时串线风险。

### 结果

现在会等待新目标对应的视觉 pose 和语义真正到达后，再触发生成。

---

## 修改 6：标定文件 fallback 路径修正

### 修改内容

`arm_pose_controller.py::load_calibration()` 的 fallback 从：

- `share/hand_eye_calibration/config/calib.yaml`

改为：

- `share/hand_eye_calibration/calib.yaml`

### 为什么改

当前包的实际安装路径就是后者。旧路径会在某些场景下错误回退到单位矩阵。

---

## 本次没有继续落地的部分

以下内容在已有 4-23 记录中提到，但本次没有继续加入：

1. `preobserve -> observe` 两阶段推进
2. 执行层更强的 IK / 碰撞前置过滤
3. 更多候选相位控制

原因是本次优先处理“输入不稳”和“输出过频”这两个更基础的问题。

---

## 验证

已完成：

1. `python3 -m py_compile`
   - `src/vision_detection/vision_detection/detection_node.py`
   - `src/vision_arm_control/vision_arm_control/arm_pose_controller.py`
2. `colcon build --packages-select vision_detection vision_arm_control`

构建通过。

---

## 建议观察现象

修改后建议重点观察：

1. 低质量帧时，语义消息中是否出现 `pose_freeze_mode=full`
2. `/target_pose` 是否从“每帧刷新”收敛为“有明显变化才刷新”
3. 执行器进入 `planning/executing/queued` 时，控制层是否停止立即重发
4. 目标切换瞬间是否不再出现旧位姿误发

