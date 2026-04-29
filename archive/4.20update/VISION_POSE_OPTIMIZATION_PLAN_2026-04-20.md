# 4-20 完整优化计划（视觉位姿精度与稳定性）

日期：2026-04-20  
工作区：`qrc_hand/arm_ws`

---

## 0. 背景与目标

### 已完成（现状）
- 已打通新控制链路：视觉目标位姿可驱动 MoveIt2 + `ros2_control` 执行。
- 机械臂可到达视觉给定目标位姿。

### 当前核心问题
- `vision_detection` 位姿仍存在：
  1. 精度不足（尤其姿态角）；
  2. 帧间抖动明显；
  3. 近距时可能因管口不完整导致失锁。

### 目标（本计划）
在不改变“先检测→发布目标→机械臂执行”主流程前提下，提升单次位姿质量与执行稳定性，避免对深度点云的强依赖。

---

## 1. 关键结论（基于已讨论结果）

1. **Phase D（深度融合/RANSAC）暂不作为主线**  
   对“小、白、光滑”目标，D435i 深度点云错位严重，难以稳定支持 RANSAC。

2. **多帧扫描采样不适合当前流程**  
   当前任务是“未知目标即时识别并执行”，不适合先做空间扫描建模。

3. **主线转为纯 RGB 增强路线（D'）**  
   采用“单帧可解 + 短时稳态 + 可见性约束 + 失锁续航”策略。

---

## 2. 新版总体方案（D'）

### D'-A：单帧解算增强（不依赖扫描）
目标：在当前视角直接获得更稳的初值。

- 从“仅 rim 几何”升级为“**rim + body 联合约束**”：
  - rim 提供口面法向与中心；
  - body 提供圆柱轴向投影与滚转参考；
  - 联合估计时抑制姿态漂移（特别是滚转不稳定）。
- 椭圆拟合质量门控：
  - 残差、边缘覆盖率、轴比可信区间；
  - 低质量观测降权或拒绝发布。
- 保持浮点拟合精度，避免不必要的整数量化误差。

#### A-1. 输入/输出定义
- 输入：
  - Stage1 body 跟踪结果（`track_id`、bbox、body mask）
  - Stage2 rim mask 与椭圆拟合结果
- 输出：
  - `pose_candidate = {center_3d, normal, ellipse_2d, axis_ratio, quality_score}`
  - `quality_score \in [0,1]`（用于后续 B/C 阶段）

#### A-2. 质量评分建议（可直接编码）
定义：

$$
q = w_1 q_{rim\_area} + w_2 q_{coverage} + w_3 q_{residual} + w_4 q_{axis} + w_5 q_{body\_consistency}
$$

其中建议初值：$w_1:w_2:w_3:w_4:w_5 = 0.15:0.20:0.30:0.20:0.15$。

- $q_{rim\_area}$：rim 像素面积是否在有效区间。
- $q_{coverage}$：有效边缘点占椭圆周长比例。
- $q_{residual}$：边缘点到拟合椭圆的归一化残差（越小越好）。
- $q_{axis}$：轴比是否远离退化区（过扁/过圆都降权）。
- $q_{body\_consistency}$：法向投影与 body 轴向投影一致性。

#### A-3. 代码级动作
1. 在 `pose_estimator.py` 新增：
   - `compute_quality_score(...)`
   - `check_body_rim_consistency(...)`
2. 在 `detection_node.py` 的每个 `track_id` 结果中携带 `quality_score`。
3. 当 `quality_score < quality_min_publish`：
   - 不直接丢弃 `track_id`，仅标记为“低质量候选”（给 D'‑D 使用）。

#### A-4. 完成判据（DoD）
- 日志可输出每帧 `quality_score` 与子项分数；
- 低质量帧明显减少“突发姿态跳变”；
- 与当前基线相比，静态姿态标准差下降 ≥ 20%。

### D'-B：短时稳态（定点连拍，不扫描）
目标：不移动机械臂，仅在当前视角抑制随机噪声。

- 固定视角连续采样 5–10 帧（约 200–300ms）。
- 进行鲁棒融合（中位数/Huber 加权平均）。
- 输出融合结果作为执行目标，而非单帧瞬时结果。

> 说明：这不是“扫描采样”，只是在当前视角做短时稳态估计。

#### B-1. 触发时机
- 仅在“目标确认后、执行前”触发一次 burst 采样；
- 若系统处于实时跟随模式，可每 `N` 个发布周期触发一次轻量 burst（可选）。

#### B-2. 融合策略
- 位置：`x/y/z` 用加权中位数（权重=`quality_score`）。
- 法向：先做半球对齐，再做加权平均并归一化。
- 椭圆角：采用圆周统计（避免 $0^\circ/180^\circ$ 跳变）。

#### B-3. 异常帧处理
- 若 burst 内有效帧数 `< burst_min_valid_count`，则：
  - 回退到 `last_valid_pose`；或
  - 保守发布（仅位置/放松姿态）。
- burst 融合结果输出 `burst_quality`（用于 C 阶段决策）。

#### B-4. 代码级动作
1. 在 `detection_node.py` 增加每个 `track_id` 的短窗缓冲区：
   - `deque(maxlen=burst_sample_count)`。
2. 新增 `fuse_burst_samples(samples)`；
3. 发布逻辑由“单帧 `live_pose`”改为“`fused_pose` 优先，单帧兜底”。

#### B-5. 完成判据（DoD）
- 不增加明显控制延迟（新增延时目标 < 300ms）；
- 静态位置标准差较基线下降 ≥ 30%；
- 姿态角瞬时尖峰次数（|Δθ|>阈值）下降 ≥ 50%。

### D'-C：可见性约束执行（防近距失锁）
目标：避免进入“管口不完整”死区。

- 接近阶段增加可见性判据：
  - 椭圆中心距图像边界 > `margin_px`；
  - 椭圆长轴像素长度在可解区间 `[L_min, L_max]`。
- 不满足判据时：
  - 禁止继续前进；
  - 仅允许小幅姿态修正或横向修正，优先恢复完整口沿。

#### C-1. 可见性指标定义
- `border_ok`：椭圆中心到四边最小距离 > `rim_border_margin_px`。
- `scale_ok`：椭圆长轴在 `[ellipse_major_min_px, ellipse_major_max_px]`。
- `shape_ok`：轴比在 `[axis_ratio_valid_min, axis_ratio_valid_max]`。

定义可见性状态：
- `VISIBLE_GOOD`：`border_ok & scale_ok & shape_ok`
- `VISIBLE_RISKY`：仅满足部分条件
- `VISIBLE_BAD`：大部分不满足

#### C-2. 执行动作映射
- `VISIBLE_GOOD`：允许正常推进与全姿态执行。
- `VISIBLE_RISKY`：仅允许横向微调/姿态放松，不允许前进。
- `VISIBLE_BAD`：冻结当前目标推进，回退到上一个安全观测位姿。

#### C-3. 代码级动作
1. 在发布消息中附带 `visibility_state`（或写入状态话题）。
2. 在执行侧根据 `visibility_state + quality_score` 选择动作级别。
3. 增加“连续坏帧计数”保护：
  - 连续 `k` 帧 `VISIBLE_BAD` 触发暂停并请求重新识别。

#### C-4. 完成判据（DoD）
- 近距场景中“口沿出画导致失锁”次数下降 ≥ 50%；
- 执行端无“低可见性仍持续前冲”的危险动作；
- 失败场景可回退、可恢复，不进入死循环。

### D'-D：失锁续航（rim 丢失不立即中断）
目标：短时遮挡/反光不导致流程崩溃。

- 为每个 `track_id` 引入：
  - `last_valid_pose`；
  - `lost_counter`；
  - `quality_score`。
- 当 rim 临时缺失但 body 仍在：
  - 允许 5–10 帧预测保持（低置信度发布）；
  - 同时触发执行层降级（仅位置或放松姿态）。

### D'-E：置信度驱动动作降级
目标：坏观测不驱动高风险动作。

设视觉质量分数为 $q\in[0,1]$，分级执行：

$$
q < q_1 \Rightarrow \text{冻结姿态，仅位置更新或暂停推进}
$$

$$
q_1 \le q < q_2 \Rightarrow \text{放松姿态容差执行}
$$

$$
q \ge q_2 \Rightarrow \text{全姿态执行}
$$

---

## 3. 分阶段落地计划

为避免命名歧义，映射关系如下：
- 本节 `Phase 1`：优先落地 D'-A（质量评分）+ D'-B（短时稳态）基础版
- 本节 `Phase 2`：深化 D'-A（rim+body 联合约束）
- 本节 `Phase 3`：落地 D'-C（可见性约束）并联动 D'-E（降级执行）

## Phase 1（1~2 天）：低风险高收益改造

### 目标
先显著降抖并减少异常跳变。

### 任务
1. 增加观测质量评分 `quality_score`（rim 面积、椭圆残差、边缘覆盖率、轴比合理性）。
2. 引入短时连拍融合（默认 7 帧）。
3. 新增 `last_valid_pose + lost_counter` 续航机制。
4. 参数化阈值并可在线调参。

### 预期收益
- 抖动显著下降；
- 临时 rim 丢失不立即失效。

---

## Phase 2（2~4 天）：几何增强与联合约束

### 目标
提升姿态与滚转稳定性。

### 任务
1. 结合 body 轴向信息参与姿态估计。
2. 完善椭圆拟合鲁棒性（局部弧段可拟合、先验约束）。
3. 对法向做球面插值或等效角度平滑，减少角度抖动。

### 预期收益
- 姿态角抖动减少；
- 近距时解算可用性提升。

---

## Phase 3（1~2 天）：执行层协同与防失锁策略

### 目标
实现“看不稳不盲动”。

### 任务
1. 落地 `q` 分级执行策略；
2. 增加可见性约束（防止口沿出画）；
3. 在近距阶段启用保守推进策略。

### 预期收益
- 降低近距失败率；
- 提高任务闭环成功率。

---

## 4. 代码改动建议（首轮最小集）

优先修改：
- `src/vision_detection/vision_detection/detection_node.py`
- `src/vision_detection/vision_detection/pose_estimator.py`
- `src/vision_detection/vision_detection/config.py`

首轮不改动主控制链与 MoveIt 执行拓扑，仅通过视觉质量与发布策略影响执行输入。

---

## 5. 新增参数建议

```text
# 质量评估
quality_min_publish
quality_min_full_pose
ellipse_residual_max
rim_coverage_min
axis_ratio_valid_min

# 连拍融合
burst_sample_count
burst_window_ms
burst_method  # median / huber_mean

# 失锁续航
max_lost_frames
lost_pose_decay_alpha

# 可见性约束
rim_border_margin_px
ellipse_major_min_px
ellipse_major_max_px

# 降级策略
q1_relax_threshold
q2_fullpose_threshold
```

---

## 6. 验证方案与验收指标

### A. 静态稳定性（单目标，固定视角）
- 采样 30 秒：
  - 位置标准差：$\sigma_x,\sigma_y,\sigma_z$
  - 姿态标准差：$\sigma_\theta$

### B. 抗丢失能力（近距/高光）
- 人为制造 1~5 帧 rim 缺失：
  - 是否保持连续输出；
  - 是否触发降级且不发生剧烈跳变。

### C. 执行成功率（联调）
- 连续 20 次目标触发：
  - 规划成功率；
  - 到位误差；
  - 近距失锁率。

### 目标门槛（建议）
- 抖动相对当前版本下降 ≥ 40%；
- 近距失锁导致任务失败比例下降 ≥ 50%；
- 联调成功率稳定提升。

---

## 7. 风险与回退

### 风险
- 质量门控过严导致可用率下降；
- 连拍窗口过大导致响应滞后。

### 回退策略
- 保留旧参数组与旧发布路径；
- 各新功能均通过参数开关启停；
- 分阶段上线：先观测、再放开自动执行。

---

## 8. 结论

后续主线明确为：

**放弃深度主导 → 强化纯 RGB（联合约束 + 短时稳态 + 可见性约束 + 失锁续航 + 置信度降级）**。

该路线与当前工程流程兼容，不依赖扫描，也不要求新增硬件，可在现有代码基础上逐步落地。

---

## 9. 原始 Phase A/B/C 技术路线对照（明确保留）

> 说明：以下为最初提出的 A/B/C 技术主线，已在本计划中保留；其中高复杂项按“先易后难”排期。

### Phase A（先做，低风险高收益）
1. 椭圆拟合保持浮点精度（避免 `int32` 量化损失），并增加“拟合质量分数”门控：
  - 指标：覆盖率、残差、轴长稳定性；
  - 位置：`refine_ellipse_from_hull()`（`pose_estimator.py`）。
2. `RIM_MASK_DILATION` 从固定值改为随 bbox 尺寸自适应：
  - 位置：`config.py`、`detection_node.py`。
3. Stage2 增加上一帧 ROI 预测约束（仅局部搜索），降低掩膜抖动传播。

### Phase B（提升精度的关键）
1. 将“轴比反解姿态”升级为“圆在透视下的椭圆反演 + 非线性重投影优化（LM）”。
2. 在已知半径条件下，将目标函数改为：最小化轮廓点到投影圆锥曲线误差。
3. 加入去畸变流程（D435i 畸变参数参与），降低边缘区域系统偏差。

### Phase C（显著降抖）
1. 时序滤波升级为“质量加权 One-Euro / EKF”：
  - 状态：`[x, y, z, nx, ny, nz]`；
  - 观测噪声：由拟合质量动态调节。
2. 对 `normal` 使用 `slerp`（球面插值）替代线性均值，降低角度抖动。

### 与当前执行顺序的关系
- 立即落地：Phase A（工程风险低、收益快）。
- 稳定后推进：Phase B（精度上限提升，开发复杂度高）。
- 并行插入：Phase C 可在 A 后即开始（先 One-Euro + `slerp`，再评估是否升级 EKF）。
