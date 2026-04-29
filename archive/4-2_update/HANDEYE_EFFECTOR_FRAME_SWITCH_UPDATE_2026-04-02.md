# Hand-Eye 末端帧切换更新（Joint6 -> Empty_Link6）（2026-04-02）

## 1. 背景与目标

本次改动针对 handeye 标定链中的末端帧语义一致性问题进行收敛，目标为：

1. 将 easy_handeye2 的默认末端帧由 `Joint6` 切换为 URDF 主链真实末端 link `Empty_Link6`。
2. 保留 `Joint6` 历史配置为“可回溯注释”，避免直接删除导致后续定位困难。
3. 降低“自定义 TF 末端帧”与“URDF 末端 link”混用带来的系统性误差风险。
4. 为下一智能体提供可执行的交接规范（先计划，后执行）。

---

## 2. 问题发现与原因分析

### 2.1 现象
- 运行 handeye 过程中可见两类帧语义并存：
  - 一类是由控制节点动态发布的 `base_link -> Joint6`；
  - 一类是 URDF 主链中的 `base_link -> ... -> Empty_Link6`。
- 在日志中可见 `Joint6` 被当成 frame 直接使用，但在 URDF 中 `Joint6` 本质是关节名，不是 link 名。

### 2.2 风险
- 标定的 `robot_effector_frame` 若绑定到“非 URDF 原生末端语义”（如自定义的 `Joint6` 帧），后续与模型、MoveIt、评估工具对齐时会出现解释歧义。
- 五种求解器结果差异偏大时，这类帧语义不一致会放大排障复杂度。

### 2.3 结论
- 以 URDF 结构为准，`Empty_Link6` 才是机械臂主链末端 link。
- handeye 默认应统一到 `Empty_Link6`，`Joint6` 仅作为历史兼容注释保留。

---

## 3. 已实施改动

### 3.1 修改文件

1) `arm_ws/src/hand_eye_calibration/launch/easy_handeye_d1.launch.py`
- 变更：
  - `robot_effector_frame` 默认值由 `Joint6` 改为 `Empty_Link6`。
  - 保留 `Joint6` 为注释，标注“历史配置，仅回溯”。

2) `arm_ws/src/hand_eye_calibration/launch/easy_handeye_d1_calibrate.launch.py`
- 变更：
  - include easy_handeye2 calibrate 时，`robot_effector_frame` 由 `Joint6` 改为 `Empty_Link6`。
  - 原 `Joint6` 行以注释形式保留。

### 3.2 未改动项
- `publish` launch 文件无需改动（只依赖 `name` 加载标定结果）。
- 本次未修改控制节点 `pose_mover` 的 `base_link -> Joint6` 发布逻辑，仅完成 handeye 配置层统一。

---

## 4. 验证建议（执行顺序）

### V1. 参数核对
- 启动 calibrate 后确认参数：
  - `robot_effector_frame = Empty_Link6`
  - `robot_base_frame = base_link`
  - `tracking_base_frame = camera_link`
  - `tracking_marker_frame = calib_board`

### V2. TF 连通性
建议逐条检查：
1. `base_link -> Empty_Link6`
2. `Empty_Link6 -> camera_link`（publish 后）
3. `camera_link -> calib_board`
4. `base_link -> calib_board`

### V3. 结果一致性
- 与历史 `Joint6` 方案做 2~3 轮重复标定对比：
  - 平移两两差建议 < 2~3 cm
  - 旋转两两差建议 < 5°
- 若一致性显著提升，说明末端帧语义统一有效。

---

## 5. 当前状态评估

1. 末端帧默认值已收敛到 URDF 真实末端 link，语义更一致。
2. 历史 `Joint6` 配置可回溯，不影响紧急回滚。
3. 仍需通过 2~3 轮重复标定数据验证最终收益（避免仅凭单次结果下结论）。

---

## 6. 给下一智能体的交接说明（强约束）

## 6.1 工作方式约束
下一智能体在任何修改前必须遵守：

1. **先提交计划，再执行改动**。
2. 计划必须明确：
   - 改哪些文件；
   - 每一处改动的理由；
   - 风险点与回滚方式；
   - 如何验证改动有效；
   - 对现有流程的影响范围。
3. 只有在用户明确回复“批准执行”后，才允许开始修改。

## 6.2 计划内容要求（必须“详尽理由”）
- 不能只写“把 A 改成 B”；必须说明“为什么这一步必要、为什么不是其他方案”。
- 对每个参数（如 effector frame、tracking frame）必须解释其 TF 拓扑含义与误差影响。
- 若涉及历史兼容，必须保留注释/回溯路径并说明回滚命令。

## 6.3 优先任务建议
1. 完成 `Empty_Link6` 方案的 2~3 轮重复性评估并形成表格。
2. 若仍存在明显分歧，再评估是否需要引入 `tool0` 固定子帧（挂在 `Empty_Link6` 下）。
3. 保持 `camera_link` 作为 tracking base，避免恢复到 optical frame 直接发布导致多父冲突。

---

## 7. 复盘命令清单（建议）

1. 校验 TF：
- `ros2 run tf2_ros tf2_echo base_link Empty_Link6`
- `ros2 run tf2_ros tf2_echo Empty_Link6 camera_link`
- `ros2 run tf2_ros tf2_echo camera_link calib_board`
- `ros2 run tf2_ros tf2_echo base_link calib_board`

2. 标定：
- `ros2 launch hand_eye_calibration easy_handeye_d1_calibrate.launch.py`

3. 发布：
- `ros2 launch easy_handeye2 publish.launch.py name:=d1_d435i_eih_camlink`

4. 评估：
- `ros2 launch easy_handeye2 evaluate.launch.py name:=d1_d435i_eih_camlink`

---

## 8. 本次结论

本次变更属于“配置语义收敛”，并非算法替换。核心收益是：
- handeye 末端定义与 URDF 主链一致；
- 降低后续排错歧义；
- 为重复性评估提供更稳的基线。

---

## 9. 聊天全程代码改动汇总（完整）

> 说明：本节汇总本轮会话中所有已经落地到工作区的代码/配置变更，不限于“末端帧切换”。

### 9.1 easy_handeye2 侧修补

1) evaluator 日志对象修复（避免二次异常刷屏）
- 文件：`easy_handeye2/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_rqt_evaluator_widget.py`
- 改动：异常分支中 `self.node` 全部改为 `self._node`。
- 原因：`self.node` 不存在，ConnectivityException 后会触发 AttributeError，掩盖主因并污染日志。
- 状态：已完成并通过语法检查。

### 9.2 hand_eye_calibration 侧修补

2) 棋盘 TF 时间戳策略调整
- 文件：`arm_ws/src/hand_eye_calibration/config/chessboard_tf_publisher.yaml`
- 改动：`use_ros_now_stamp: true -> false`。
- 原因：优先使用图像时间戳，降低“图像时刻与机器人TF时刻不一致”导致的采样时序误差。
- 状态：已完成并通过配置校验。

3) 增补 easy_handeye 包装 launch（source 层补齐）
- 新增文件：
  - `arm_ws/src/hand_eye_calibration/launch/easy_handeye_d1_calibrate.launch.py`
  - `arm_ws/src/hand_eye_calibration/launch/easy_handeye_d1_publish.launch.py`
- 原因：source 目录原先缺失这两个入口，而 install 中存在；补齐后 source/install 结构一致，便于后续维护与复现。
- 状态：已完成并通过语法检查。

### 9.3 末端帧语义收敛（本记录主题）

4) handeye 默认末端从 Joint6 切换为 Empty_Link6（并保留注释回溯）
- 文件：
  - `arm_ws/src/hand_eye_calibration/launch/easy_handeye_d1.launch.py`
  - `arm_ws/src/hand_eye_calibration/launch/easy_handeye_d1_calibrate.launch.py`
- 改动：
  - `robot_effector_frame` 默认/传参值改为 `Empty_Link6`；
  - `Joint6` 作为历史配置注释保留。
- 原因：`Empty_Link6` 是 URDF 主链真实末端 link，`Joint6` 在 URDF 中为关节名；统一语义可减少排障歧义。
- 状态：已完成并通过语法检查。

---

## 10. 计划执行情况复盘（全过程）

### 10.1 已执行流程类型

本次会话存在两类执行阶段：

1. **前期快速修补阶段**（先排查后直接修补）
- 执行了 evaluator 修补、时间戳策略调整、source launch 补齐。
- 特征：以“先定位问题并立即降低风险”为主。

2. **后期受控执行阶段**（先计划，获批后执行）
- 在用户明确提出流程约束后，先提交计划；
- 用户回复“批准执行”后，才实施 `Joint6 -> Empty_Link6` 切换与本记录文档落地。

### 10.2 与用户要求的一致性状态

- 当前状态：已切换到“先计划后执行”工作模式，并已在本记录和交接约束中固化。
- 仍需保持：后续任何新增修改都必须先给出详细计划与理由，等待用户批准。

### 10.3 每项计划的落地结果

1) 计划项：定位 handeye 入口文件并统一末端帧
- 结果：已完成（2 个 launch 文件已切换）。

2) 计划项：保留 Joint6 回溯注释
- 结果：已完成（代码内保留注释）。

3) 计划项：最小改动验证
- 结果：已完成（相关文件未发现语法/配置错误）。

4) 计划项：生成标准化交接文档
- 结果：已完成（本文件 + 约束条款齐备）。

---

## 11. 下一智能体启动前检查清单（执行前必须先给计划）

1. 先提交计划（禁止直接改动），并逐项写明理由：
   - 为什么要改这几个文件；
   - 为什么是该方案而不是备选方案；
   - 风险点与回滚路径；
   - 验证步骤与通过标准。
2. 获得用户“批准执行”后再修改。
3. 修改后必须更新本记录的“代码改动汇总”和“计划执行复盘”。

---

## 12. 后续建议（基于当前会话结果）

1. 先按 `Empty_Link6` 路线做 2~3 轮重复标定，比较轮间一致性。
2. 若求解器间仍有明显分歧，再评估是否引入 `tool0` 固定子帧（挂在 `Empty_Link6` 下）以更贴近真实安装位姿。
3. 继续保持：
   - `tracking_base_frame = camera_link`
   - 不与 D435i optical frame 形成 child 多父冲突。
