# Hand-Eye 标定与 TF 集成阶段更新（2026-04-02）

## 1. 背景与目标

本阶段围绕 Unitree D1-550 + D435i 的手眼标定集成问题展开，核心目标为：

1. 明确并验证 easy_handeye2 依赖的两条 TF 链路是否稳定可用。
2. 将旧自研标定流程降级为“可回溯但不干扰当前流程”的状态（Phase B）。
3. 诊断 evaluate 过程中 TF 断树与日志异常问题。
4. 修复相机发布目标帧选择不当导致的 TF 冲突（`camera_color_optical_frame` 冲突）。
5. 建立后续可复现、可维护的标定与评估操作规范。

---

## 2. 初始系统状态与已知约束

### 2.1 当前系统形态
- 机械臂链路：`base_link -> Joint6` 由 `pose_mover` 动态发布。
- 视觉链路：`camera_* -> calib_board` 由棋盘检测节点发布（`chessboard_tf_publisher_node`）。
- 手眼工具：使用 `easy_handeye2` 进行 `eye_in_hand` 标定。

### 2.2 关键约束
- D435i 驱动已发布完整相机静态 TF：
  - `camera_link -> camera_color_frame -> camera_color_optical_frame`
  - `camera_link -> camera_depth_frame -> camera_depth_optical_frame`
- 在 ROS TF 中，单一 child frame 不应被多个不同 parent 同时发布，否则会出现树结构冲突。

---

## 3. 本阶段发现的问题（按时间线）

### 3.1 `hand_eye_calibration` 编译失败（误导性长报错）
- 现象：`colcon build --packages-select hand_eye_calibration` 报大量 STL/Eigen/rclcpp 模板错误。
- 根因：`hand_eye_calibration_node.cpp` 首行存在非法字符前缀：`0#include <memory>`。
- 结论：后续连锁错误均为首个语法错误扩散，不是依赖库本身问题。

### 3.2 Phase B 过渡中的兼容风险
- 需求：先停用自研求解节点，保留历史逻辑以便回滚。
- 风险：若仅停用可执行目标但不调整 launch，启动时仍可能引用已停用节点。
- 处理：
  - CMake 停用 `hand_eye_calibration_node` 的构建/安装。
  - 旧逻辑保留为注释。
  - `hand_eye_calibration.launch.py` 调整为只启动 `chessboard_tf_publisher_node`，旧流程注释保留。

### 3.3 evaluate 时报 `base_link <-> calib_board` 不连通
- 现象：`evaluate.launch.py` 报 `ConnectivityException`：
  - `Could not find a connection between 'base_link' and 'calib_board'`
- 代码逻辑原因：
  - 在 `eye_in_hand` 下，evaluator 计算链路依赖 `base_link -> calib_board`。
  - 若机器人链、发布链、视觉链任一缺失即报错。

### 3.4 evaluator 二次异常导致日志刷屏
- 现象：原始 ConnectivityException 后继续抛 `AttributeError`。
- 根因：异常分支中使用了 `self.node`（不存在），应为 `self._node`。
- 结果：同一问题被重复刷屏，增加诊断噪声。

### 3.5 标定结果发布后 TF 仍不稳定连通（核心问题）
- 现象：`publish` 节点运行，但 `Joint6 -> camera_color_optical_frame` 与评估链路偶发断树。
- 核心根因：标定目标帧选为 `camera_color_optical_frame`，与 D435i 内建静态链重复占用 child frame。

具体冲突关系：
1. easy_handeye 发布：`Joint6 -> camera_color_optical_frame`
2. D435i 固有发布：`camera_color_frame -> camera_color_optical_frame`

同一个 child (`camera_color_optical_frame`) 出现两个 parent，TF 树被破坏，evaluate 连通性不稳定。

### 3.6 评估 UI 提示“move around”但移动 marker 无效
- 原因：evaluator 的“新样本判定”主要基于机器人位姿变化（`base_link -> Joint6`），而不是仅 marker 变化。
- 结论：应固定标定板、移动机械臂采样；仅移动板不会通过“新姿态”判定。

---

## 4. 排查过程与关键证据

### 4.1 TF 验证流程
本阶段反复验证以下链路：
1. `base_link -> Joint6`
2. `Joint6 -> camera_*`
3. `camera_* -> calib_board`
4. `base_link -> calib_board`

在冲突阶段可观察到：
- 单条链路局部可查到，但整树连通关系不稳定。
- `/tf_static` 中出现 `camera_color_optical_frame` 多父关系。

### 4.2 D435i 原生相机 TF 结构确认
确认相机本体结构正确，关键静态链路为：
- `camera_link -> camera_depth_frame -> camera_depth_optical_frame`
- `camera_link -> camera_color_frame -> camera_color_optical_frame`

结论：冲突来自 handeye 发布目标选择，不是 D435i 驱动异常。

---

## 5. 已实施修复与过渡策略

### 5.1 Phase B（保留旧逻辑注释）
- `hand_eye_calibration` 包仅保留棋盘 TF 发布功能参与主流程。
- 旧自研标定节点、旧 launch 逻辑保留注释，不做物理删除。

### 5.2 手眼发布目标帧修正策略
为避免与 D435i optical frame 冲突，采用：
- 发布目标从 `camera_color_optical_frame` 改为 `camera_link`。

执行方式：
1. 读取现有 `d1_d435i_eih` 标定结果。
2. 利用 D435i 内置静态关系，将 `Joint6 -> camera_color_optical_frame` 等价换算为 `Joint6 -> camera_link`。
3. 生成新标定名：`d1_d435i_eih_camlink`。

生成文件：
- `/home/arnoyin/.ros2/easy_handeye2/calibrations/d1_d435i_eih_camlink.calib`

### 5.3 运行时注意事项
- 不要长期同时运行 `calibrate.launch.py` 与 `publish.launch.py`。
- `publish` 节点前台“停在 loading 日志”是正常现象（进入 spin 事件循环）。

---

## 6. 当前结果评估

### 6.1 连通性
- 在 `camlink` 方案下，`base_link -> calib_board` 已可跑通（关键目标达成）。

### 6.2 精度信号
- evaluator 显示约 `0.19`（平移分散度量级约 19 cm）属于高误差，当前标定质量不理想。
- 该值虽非完整位姿误差，但可作为“当前结果不可靠”的警示。

### 6.3 质量风险归因
高误差可能来自：
1. 样本数偏少（仅约 10 点）。
2. 姿态覆盖不足或不均匀。
3. 采样时序中存在瞬时不稳。
4. 历史冲突配置残留造成样本一致性下降。

---

## 7. 本阶段结论

1. 问题核心不是 easy_handeye 算法本身，而是 TF 发布拓扑冲突与采样策略偏差。
2. `tracking_base_frame` 直接选 `camera_color_optical_frame` 在当前系统下存在高冲突风险。
3. 迁移到 `camera_link` 作为发布目标后，系统连通性显著改善。
4. 当前精度指标仍不达标，需要以“固定标定板 + 机械臂多旋转低平移冗余采样”重采。

---

## 8. 后续建议（下一步执行顺序）

### P1. 固化配置
1. 后续所有新标定统一使用：
   - `tracking_base_frame = camera_link`
   - `tracking_marker_frame = calib_board`
2. 保留 D435i 原生 optical 链，禁止重复发布 `camera_color_optical_frame` 为 child。

### P2. 重新采样
1. 样本数提升至 20~30。
2. 优先增加旋转覆盖（X/Y/Z 正负方向），平移保持小范围。
3. 每点到位后等待 0.5~1.0s 再采样。

### P3. 工具修补（建议）
- 修复 evaluator 中日志对象误用：`self.node` -> `self._node`，消除刷屏噪声。

### P4. 验证闭环
1. `publish` 后验证 4 条链路连通。
2. `evaluate` 观察误差，目标至少进入厘米级，再进一步优化到毫米~低厘米级。

---

## 9. 关键命令清单（复盘用）

1. 连通性检查：
   - `ros2 run tf2_ros tf2_echo base_link Joint6`
   - `ros2 run tf2_ros tf2_echo Joint6 camera_link`
   - `ros2 run tf2_ros tf2_echo camera_link calib_board`
   - `ros2 run tf2_ros tf2_echo base_link calib_board`

2. 发布标定：
   - `ros2 launch easy_handeye2 publish.launch.py name:=d1_d435i_eih_camlink`

3. 评估：
   - `ros2 launch easy_handeye2 evaluate.launch.py name:=d1_d435i_eih_camlink`

---

## 10. 交接提示词（用于后续会话）

> 当前系统已完成 handeye 发布目标从 optical frame 到 camera_link 的迁移试运行。接下来以 `d1_d435i_eih_camlink` 作为主配置继续重采与评估。重点关注：
> 1) TF 树单父约束（避免 optical frame 冲突）；
> 2) 20~30 点旋转覆盖采样；
> 3) evaluator 误差从 0.19 下降到可用区间；
> 4) 需要时修复 `handeye_rqt_evaluator_widget.py` 的 `self.node` 日志错误。
