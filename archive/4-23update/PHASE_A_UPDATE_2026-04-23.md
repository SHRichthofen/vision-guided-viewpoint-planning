# Phase A 更新记录（2026-04-23）

## 本次目标

按照 `4_22gpt_plan/频繁发布+目标退化` 中的计划，优先对控制层 `arm_pose_controller.py` 落地 Phase A 稳定化修改，先解决：

1. 高频重复发布目标导致下游规划堆积
2. `radius=0 / azimuth=180 / axial=0` 退化候选持续霸榜
3. 候选一步冲最终观察位姿，导致规划失败或被放宽到无意义位置

本次只修改控制层与其参数文件，不扩展到执行器或视觉识别算法本体。

---

## 修改文件

1. `src/vision_arm_control/vision_arm_control/arm_pose_controller.py`
2. `src/vision_arm_control/config/arm_pose_controller_bottom_scan.params.yaml`

---

## 修改点 1：发布门控（publish gating）

### 修改位置

- `arm_pose_controller.py`
  - `__init__()` 中新增参数读取：
    - `min_publish_interval_sec`
    - `min_position_delta_m`
    - `min_orientation_delta_rad`
    - `min_score_improvement`
  - 新增函数：
    - `_should_publish_target()`
    - `_request_target_generation()`
  - `generate_and_publish_target()` 中接入门控判断

- `arm_pose_controller_bottom_scan.params.yaml`
  - 新增对应参数默认值

### 为什么修改

之前控制层只要收到新的 `/cylinder_pose_base` 或视觉语义更新，就会立刻重新生成候选并发布目标。视觉是连续发布的，这会导致：

1. 上游每一帧都想改目标
2. 下游执行器虽然做了 latest-wins，但仍会长期处于 planning / queued / executing 循环
3. 真实问题还没定位清楚，就先被高频重发放大成“系统卡死”

### 修改后的行为

只有在以下任一条件满足时，控制层才会重新发布目标：

1. 距离上次发布超过最小时间间隔
2. 新目标与上次目标的位置差明显增大
3. 新目标与上次目标的姿态差明显增大
4. 新候选分数明显优于上次
5. 发生了强制发布事件（例如目标切换、阶段切换）

### 预期效果

先把“视觉连续更新”收敛成“控制层按阈值发关键目标”，减轻执行器被重复刷新的问题。

---

## 修改点 2：执行器忙闲门控（busy gating）

### 修改位置

- `arm_pose_controller.py`
  - `__init__()` 中新增：
    - `exec_status_topic`
    - `busy_status_states`
    - `/vision_exec_status` 订阅
  - 新增函数：
    - `on_executor_status()`
    - `_executor_is_busy()`
    - `_request_target_generation()`

- `arm_pose_controller_bottom_scan.params.yaml`
  - 新增：
    - `exec_status_topic`
    - `busy_status_states`

### 为什么修改

之前控制层完全不知道执行器当前是否正在 planning / executing / queued。结果是：

1. 执行器在忙
2. 控制层继续按视觉更新频繁发新目标
3. 整个系统表现成“还没做完上一条，就被下一条覆盖”

这会把原本应该在控制层解决的问题，错误地下沉成执行层拥塞问题。

### 修改后的行为

当执行器状态属于 `planning / executing / queued` 时：

1. 控制层不立即发布新目标
2. 只缓存“最新一份视觉更新请求”
3. 等执行器回到非忙碌状态后，再根据最新视觉结果重新判断是否要发布

### 预期效果

让控制层和执行器形成“上游有节制、下游可消化”的串行协作关系，而不是双向打架。

---

## 修改点 3：去除零半径退化候选，并修正评分机制

### 修改位置

- `arm_pose_controller.py`
  - `__init__()` 中新增：
    - `allow_zero_radius_candidates`
  - 新增函数：
    - `_phase_candidate_settings()`
  - 修改函数：
    - `_candidate_score()`
    - `_generate_bottom_ring_candidates()`

- `arm_pose_controller_bottom_scan.params.yaml`
  - `candidate_radius_list_m` 从 `"0.0,0.015,0.03"` 改为 `"0.015,0.03"`
  - 新增 `allow_zero_radius_candidates: false`

### 为什么修改

旧版评分存在两个直接问题：

1. 候选列表包含 `radius=0`
2. 评分函数中的旧 `front_term` 会对中间索引产生隐式偏置，均匀采样时很容易把分数推向 `180°`

结果就是日志里反复出现：

- `radius = 0.0000`
- `azimuth = 180deg`
- `axial = 0.0000`

这不是偶然采样到坏点，而是评分函数把退化候选稳定选成了“最优”。

### 修改后的行为

1. 默认不再生成零半径候选
2. 去掉与 azimuth 索引绑定的人工偏置
3. 改为基于“非零侧向观察半径”和“阶段目标半径”进行评分
4. 保留 `quality_score`、`near_circle` 对搜索尺度的影响

### 预期效果

先从候选排序层面打掉“中心线直冲 + 180 度锁死”的退化最优解。

---

## 修改点 4：preobserve -> observe 两阶段推进

### 修改位置

- `arm_pose_controller.py`
  - `__init__()` 中新增：
    - `preobserve_enabled`
    - `preobserve_standoff_extra_m`
    - `preobserve_radius_scale`
    - `preobserve_min_radius_m`
    - `preobserve_axial_scale`
  - 新增函数：
    - `_phase_name()`
    - `_radius_target_norm()`
    - `_phase_candidate_settings()`
  - `on_target_id_callback()` 中新增阶段重置
  - `on_executor_status()` 中新增阶段切换逻辑
  - `_generate_bottom_ring_candidates()` 中按阶段生成不同风格的候选

- `arm_pose_controller_bottom_scan.params.yaml`
  - 新增对应 preobserve 参数

### 为什么修改

旧逻辑默认一步冲最终观察位姿，导致两个问题：

1. 目标可能过于激进，不容易规划
2. 为了让规划成功，只能继续放宽 MoveIt 约束，最后就容易“成功到一个很离谱的位置”

### 修改后的行为

1. 新目标先进入 `preobserve`
   - 半径更大
   - standoff 更远
   - axial 偏移更保守
2. 当执行器上报 `preobserve` 成功后，控制层自动切换到 `observe`
3. 再生成更接近最终观察任务的候选位姿

### 预期效果

先拿到一个更容易到达、至少朝向合理的预观察位姿，再逼近最终观察位姿，降低“一步到位失败”的风险。

---

## 额外修正：标定文件 fallback 路径

### 修改位置

- `arm_pose_controller.py`
  - `load_calibration()` 中 fallback 从 `share/hand_eye_calibration/config/calib.yaml`
    改为 `share/hand_eye_calibration/calib.yaml`

### 为什么修改

当前 `hand_eye_calibration` 的安装规则是把 `calib.yaml` 安装到：

- `share/hand_eye_calibration/calib.yaml`

旧 fallback 路径和实际安装位置不一致，可能在未显式传 `calib_file` 时回退到单位矩阵。

### 预期效果

减少标定文件路径不一致导致的隐式回退风险。

---

## 当前未改动的部分

本次没有修改：

1. `vision_moveit_executor.cpp` 的前置 IK / 碰撞预筛
2. 视觉层检测频率
3. 深度先验算法
4. 自定义消息结构

这些仍属于后续 Phase B / Phase C 范围。

---

## 本次改动后建议验证的现象

1. 控制层日志中不应再持续高频刷新同一个目标
2. 最佳候选不应再长期固定为 `radius=0`
3. `azimuth=180deg` 不应再因旧评分偏置而持续霸榜
4. 任务开始时应先看到 `preobserve`
5. 执行器 `success` 后，控制层应能切到 `observe`

---

## 后续建议

如果 Phase A 后仍然出现：

1. 候选朝向明显不对
2. 规划成功但相机没真正看向圆柱
3. 仍然大量“goal tree 无法采样”

则下一步应进入 Phase B：

1. 在执行层增加更强的可达性/IK 前置过滤
2. 进一步减少明显不可达候选的实际规划尝试
3. 区分“目标位姿生成错误”和“规划器假阳性成功”
