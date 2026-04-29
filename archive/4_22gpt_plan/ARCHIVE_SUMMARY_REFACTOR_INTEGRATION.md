# 重构接入留档总结

## 1. 本次新增文件

### 控制包 `control`
- `control/config/vision_moveit_executor.params.yaml`
- `control/launch/run_new_control_refactor.launch.py`
- `control/launch/vision_bottom_scan_bringup.launch.py`

### 视觉-控制包 `vision_arm_control`
- `vision_arm_control/config/arm_pose_controller_bottom_scan.params.yaml`
- `vision_arm_control/launch/vision_arm_integration_refactor.launch.py`

### 检测包 `vision_detection`
- `vision_detection/config/cylinder_detection_refactor.params.yaml`

---

## 2. 本次替换的核心源码
- `vision_detection/vision_detection/detection_node.py`
- `vision_detection/vision_detection/config.py`
- `vision_arm_control/vision_arm_control/arm_pose_controller.py`
- `control/src/vision_moveit_executor.cpp`

---

## 3. 本次改动的目的

### 3.1 视觉层
从“只发单个管口 pose”升级为：
- 连续发布目标位姿
- 发布质量与退化语义 `/vision/selected_cylinder_semantics`
- 为闭环观察留出通道

### 3.2 控制层
从旧的 `axis_guided_view` 单目标生成，升级为：
- `bottom_ring_candidates` 候选观察位姿生成
- 发布 `/target_pose_candidates`
- 发布 `/target_pose_debug`

### 3.3 执行层
从“只消费 `/target_pose` 单目标”升级为：
- 优先消费 `/target_pose_candidates`
- batch 排序 + 候选尝试
- busy 时缓存最新 batch，而不是只缓存单个 pose

### 3.4 启动架构
从“两条链分别启动”升级为：
- 视觉链 refactor launch
- MoveIt/ros2_control refactor launch
- 顶层统一 bringup launch

---

## 4. 为什么要这样改
旧系统的核心问题不是单个 planner 参数，而是：
1. 视觉输出只有“中心+法向”，任务语义不足；
2. 控制层用单个轴向 pose 直接驱动观察任务；
3. 执行层只在坏初值附近小范围试错；
4. launch 与参数来源分裂，难以稳定复现。

这次改动的目标，是先让系统具备：
- 候选观察位姿
- 持续更新
- 候选集执行
- 统一启动入口

为下一步真正的闭环调度器打基础。

---

## 5. 后续建议
1. 先验证新 launch + 新候选链路跑通；
2. 再增加“观察任务调度器”状态机；
3. 再进一步把语义 topic 从 `String/JSON` 升级成自定义消息；
4. 最后再做视觉质量和深度先验的细化优化。
