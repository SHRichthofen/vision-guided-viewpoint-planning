# 4-19 更新日志（视觉链路适配新控制架构 + 目标姿态放松改造）

日期：2026-04-19  
工作区：`qrc_hand/arm_ws`  
主线目标：
1. 将旧视觉模块（原先面向 `pose_mover`）适配到新标准链路（MoveIt2 + `ros2_control` + `D1SystemInterface`）。
2. 解决“视觉有目标但 MoveIt 频繁 `Unable to sample any valid states for goal tree`”问题。
3. 在不丢失圆柱姿态参考（圆柱可能水平/任意倾斜）的前提下，降低目标姿态刚性，提高可规划率。

---

## 0. 本轮结论摘要

本轮完成了三类工作：

1. **新适配执行节点落地**：新增 `vision_moveit_executor`，把视觉目标位姿接入 MoveIt 标准执行链路。  
2. **视觉姿态链路稳态增强**：修复法向/四元数连续性与 NaN 问题，增加相机 optical/link 帧对齐逻辑。  
3. **目标姿态放松与轴向参考并行支持**：新增轴向参考观察模式，执行侧增加姿态容差与失败回退策略。

同时记录到：
- 老代码保留为注释（按要求未直接删除关键旧逻辑路径）。

---

## 1. 新增：视觉到 MoveIt 的适配执行节点（新直连代码）

### 1.1 新增文件

- `src/control/src/vision_moveit_executor.cpp`

### 1.2 作用

作为“视觉目标 -> MoveIt 执行”桥接层：

- 订阅：
  - `/target_pose_stamped`（主）
  - `/target_pose`（兼容）
- 执行：
  - `MoveGroupInterface::plan()` + `execute()`
- 输出：
  - `/vision_exec_status`（JSON 字符串状态）

### 1.3 关键机制

- 防抖/节流：`min_target_interval_sec`、`min_position_delta_m`、`min_orientation_delta_rad`
- 忙时保留最新目标：`queue_latest_when_busy`
- 规划参数化：`planning_time`、`num_planning_attempts`、速度/加速度缩放

---

## 2. 控制包构建与启动整合

### 2.1 修改文件

- `src/control/CMakeLists.txt`
- `src/control/launch/vision_new_control.launch.py`

### 2.2 变更内容

1. `CMakeLists.txt` 注册并安装新可执行：
   - `vision_moveit_executor`

2. 新增一键整合启动：
   - `vision_new_control.launch.py`
   - 同时拉起：
     - 新控制主链 `run_new_control.launch.py`
     - 视觉链 `vision_arm_integration.launch.py`
     - `vision_moveit_executor`

---

## 3. 视觉姿态稳态增强（Phase A-C）

### 3.1 修改文件

- `src/vision_detection/vision_detection/pose_estimator.py`
- `src/vision_detection/vision_detection/detection_node.py`
- `src/vision_arm_control/vision_arm_control/vision_to_arm_transform.py`
- `src/vision_arm_control/vision_arm_control/arm_pose_controller.py`

### 3.2 主要改动

#### A) 输入合法性与 NaN 防护

- 对 `center_3d` / `normal` 增加 `isfinite` 检查。
- 法向归一化增加零范数回退（上次法向或默认法向），防止 NaN 扩散。

#### B) 姿态连续性

- 引入四元数“半球连续化”：若 `dot(q_prev, q_now) < 0` 则取 `-q_now`。
- 引入滚转连续策略：优先使用上一帧 X 轴投影，减小绕视线随机转动。

#### C) 发布门控

- 当法向非法、四元数非法时直接跳过发布，并输出明确告警。
- 增加姿态质量日志（法向范数、四元数范数）。

---

## 4. optical/link 坐标系对齐验证与修复

### 4.1 背景

实测中出现“视觉目标语义正确但执行偏差大”，怀疑目标相机位姿按 optical 语义构造、而手眼外参按 `camera_link` 语义使用，存在帧语义不一致。

### 4.2 修改文件

- `src/vision_arm_control/vision_arm_control/arm_pose_controller.py`

### 4.3 变更内容

新增参数：
- `desired_camera_frame`（默认 `camera_color_optical_frame`）
- `calibration_camera_frame`（默认 `camera_link`）
- `use_tf_camera_frame_alignment`（默认 `true`）

新增流程：
1. 先生成 `T_ref_cam_des`（desired_camera_frame 语义）
2. 通过 TF 查询 `desired <- calibration`
3. 计算 `T_ref_cam_for_calib`
4. 再与 `T_camera_to_tool` 相乘，得到末端目标

并增加一次性日志打印对齐量（平移范数、旋转角），用于现场验证 optical/link 差异。

---

## 5. 关键问题修复：`align_desired_camera_pose_to_calibration_frame` 作用域错误

### 5.1 现象

运行时报错：

`AttributeError: 'ArmPoseController' object has no attribute 'align_desired_camera_pose_to_calibration_frame'`

### 5.2 根因

该函数曾因缩进错误被定义为 `generate_and_publish_target()` 内部局部函数，而非类成员方法。

### 5.3 处理

已将其提升为类级方法（正确缩进），恢复 `self.align_desired_camera_pose_to_calibration_frame(...)` 可调用性。

---

## 6. 姿态放松改造（保留圆柱姿态参考）

> 注意：本轮明确按需求“圆柱可水平/任意方向，不可用固定俯视替代”。

### 6.1 目标生成侧（`arm_pose_controller.py`）

新增参数：
- `target_mode`（默认 `axis_guided_view`）
- `axis_observe_offset_m`
- `camera_standoff_m`
- `roll_reference_axis`

新增模式：
- `axis_guided_view`：沿圆柱轴向构造观察点与相机视线（保留姿态参考）
- 保留旧模式逻辑为注释（兼容/回溯用途）

### 6.2 执行侧（`vision_moveit_executor.cpp`）

新增参数：
- `goal_position_tolerance`
- `goal_orientation_tolerance`
- `relaxed_orientation_retry`
- `relaxed_orientation_tolerance`

策略：
1. 先按主目标规划
2. 若失败且启用回退：
   - 保留位置目标
   - 使用当前姿态（或更宽松姿态容差）再规划一次
3. 恢复默认容差

> 旧“单次严格规划”保留为注释。

---

## 7. 启动文件参数透传

### 7.1 修改文件

- `src/vision_arm_control/launch/vision_arm_integration.launch.py`
- `src/control/launch/vision_new_control.launch.py`

### 7.2 内容

`vision_arm_integration.launch.py` 新增透传：
- `target_mode`
- `axis_observe_offset_m`
- `camera_standoff_m`
- `roll_reference_axis`
- `desired_camera_frame`
- `calibration_camera_frame`
- `use_tf_camera_frame_alignment`

`vision_new_control.launch.py` 新增执行器放松参数：
- `goal_position_tolerance=0.01`
- `goal_orientation_tolerance=0.6`
- `relaxed_orientation_retry=true`
- `relaxed_orientation_tolerance=1.2`


---

## 8. 验证建议（下一轮）

1. 参数生效核查：
   - `/arm_pose_controller`: `target_mode`, `camera_standoff_m`, `desired_camera_frame`, `calibration_camera_frame`
   - `/vision_moveit_executor`: `goal_orientation_tolerance`, `relaxed_orientation_retry`

2. 目标稳定性验证：
   - 固定目标连续触发 20 次，统计 Plan success 率
   - 对比 `axis_guided_view` 与（注释保留的）旧逻辑

3. 执行落地一致性验证：
   - 同步观察 MoveIt 成功与 `/d1_hw_exec_status` 跟踪误差变化

---

## 10. 本轮涉及文件总览

### 新增
- `src/control/src/vision_moveit_executor.cpp`
- `src/control/launch/vision_new_control.launch.py`

### 修改
- `src/control/CMakeLists.txt`
- `src/control/src/vision_moveit_executor.cpp`
- `src/control/launch/vision_new_control.launch.py`
- `src/vision_detection/vision_detection/pose_estimator.py`
- `src/vision_detection/vision_detection/detection_node.py`
- `src/vision_arm_control/vision_arm_control/vision_to_arm_transform.py`
- `src/vision_arm_control/vision_arm_control/arm_pose_controller.py`
- `src/vision_arm_control/launch/vision_arm_integration.launch.py`

---

## 11. 备注

- 本日志按“适配链路 -> 稳态增强 -> 坐标系验证 -> 姿态放松”的顺序整理。
- 关键旧逻辑均以注释形式保留，未直接删除。
- 后续若要进一步收敛“偶发 goal tree 无效”，优先从参数 A/B 与环境一致性入手，再做算法层收紧。
