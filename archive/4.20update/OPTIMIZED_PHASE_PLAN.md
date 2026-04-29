# 优化计划调整与合理化排序
**日期**: 2026-04-20  
**基于现状**: vision_detection 核心结构已完成（两阶段检测、椭圆拟合、位姿解算、平滑）

---

## 一、现状评估

### 现有能力  
✓ Stage 1/2 完整检测链路  
✓ 椭圆精修拟合 (`refine_ellipse_from_hull`)  
✓ 基础平滑 (`PoseSmoother`, 含死区与突变保护)  
✓ 聚合缓冲 (`PoseAggregator` 用于多帧确认)  
✓ 发布架构 (PoseArray + PoseStamped)  

### 现有缺陷  
✗ 无质量评分机制 → 坏观测直接发布  
✗ 无短时稳态融合 → 单帧抖动传播  
✗ 无可见性约束 → 近距失锁无防护  
✗ 无失锁续航 → rim缺失立即中断  
✗ 固定 RIM_MASK_DILATION → 尺度自适应缺失  

---

## 二、算法层面的根本约束 ⚠️

**关键发现**: 当前"轴比反解倾角"方法存在数学敏感性上限。

椭圆轴比 → 倾角的映射函数 $\theta = \arccos(r)$ 在 $r$ 接近 1 时导数趋向无穷：
- 拟合误差 ±2px → 倾角误差 ±1~3°（无法消除）
- Phase A+B 最多改善到 **±0.8°**，这是该方法的 **无法突破的天花板**

**突破方案** (可选的 Phase D)：
- 使用**非线性重投影优化** (LM优化) 替代直接映射
- 在 6DOF 位姿空间直接优化，充分利用 100+ 个轮廓点信息
- 预期精度提升 3~5 倍（±0.8° → ±0.15°）
- 代价：计算耗时 2ms → 15ms，需 2~3 天开发

**当前计划的适用范围**:
- ✓ 精度需求 ±10mm, ±5° 以内 → Phase A+B 足够
- ✗ 精密应用 ±2mm 以内 → 需要 Phase D

详见 [ALGORITHM_CONSTRAINT_ANALYSIS.md](arm_ws/archive/4.20update/ALGORITHM_CONSTRAINT_ANALYSIS.md)

---

## 三、重新排序（按实施难度与收益权衡）

### **阶段A（1天）：质量评分 + 短时稳态基础版**
**目标**: 降低抖动、避免坏观测发布

#### A1. 质量评分体系 [`pose_estimator.py`]
- 新增 `compute_quality_score(pose_dict, ellipse_metrics)` 函数
- 计算子项：
  - `q_rim_area`: 椭圆面积有效区间检查
  - `q_coverage`: 边缘覆盖率 (实际边缘点 / 椭圆周长)
  - `q_residual`: 拟合残差 (椭圆线到边缘的平均距离)
  - `q_axis`: 轴比合理性 (避免过扁/过圆)
- **加权组合**: $q = 0.20 \cdot q_{\text{area}} + 0.25 \cdot q_{\text{coverage}} + 0.35 \cdot q_{\text{residual}} + 0.20 \cdot q_{\text{axis}}$
- 在 `process_detection()` 末尾调用，返回结果中添加 `quality_score` 字段

#### A2. 发布质量门控 [`detection_node.py`]
- 新增配置参数:
  - `quality_min_publish = 0.5` (低于此不发布)
  - `quality_min_full_pose = 0.65` (低于此仅发布位置、冻结姿态)
- 在 `publish_cylinders_array()` 和 `publish_selected_target()` 前检查 `quality_score`
- 低质量时: 标记状态（供后续 D 阶段使用），但先只记日志不阻止发布（可选参数控制）

#### A3. 短时稳态融合轻量版 [`detection_node.py`]
- 为每个 `track_id` 维护窗口缓冲 `deque(maxlen=7)`
- 在 `publish_selected_target()` 前触发一次融合:
  - 位置: 加权中位数（权重 = `quality_score`）
  - 法向: 半球对齐 + 加权平均 + 归一化
  - 椭圆参数: 圆周插值 (避免0/180跳变)
- **输出**: `fused_pose` (附加 `fusion_count` 字段表示有效样本数)

#### A4. 失锁续航基础 [`detection_node.py`]
- 为每个 `track_id` 记录:
  - `last_valid_pose`: 最后一个有效观测
  - `lost_counter`: 连续缺失帧数
  - `max_lost_frames = 8` (超过此值停止发布旧位姿)
- 若当前帧 rim 解算失败但 body 仍在:
  - 使用 `last_valid_pose` + 递减质量权重发布
  - `lost_counter >= max_lost_frames` 时标记为"失连"

#### A5. 新增参数 [`config.py`]
```python
# 质量评估
QUALITY_MIN_PUBLISH = 0.5
QUALITY_MIN_FULL_POSE = 0.65
ELLIPSE_RESIDUAL_MAX = 3.0  # 像素
RIM_COVERAGE_MIN = 0.6

# 短时融合
BURST_SAMPLE_COUNT = 7
BURST_FUSION_ENABLED = True

# 失锁续航
MAX_LOST_FRAMES = 8
LOST_QUALITY_DECAY = 0.85
```

#### A6. 验收指标
- 日志输出每帧 `quality_score` 及其子项
- 抖动标准差相比基线 ↓ 25%
- 坏观测被拒发比例 ≥ 60%

---

### **阶段B（2天）：几何增强 + 自适应约束**
**目标**: 提升姿态稳定性与近距可用性

#### B1. Body-Rim 联合约束 [`pose_estimator.py`]
- 新增 `check_body_rim_consistency(body_bbox, ellipse_2d, normal)` 函数
- 比较:
  - body bbox 的垂直投影方向 vs 法向投影方向的一致性
  - 如果相差 > 阈值 → 降权该观测或使用先验校正
- 在 `process_detection()` 中调用，返回 `consistency_score`

#### B2. 椭圆拟合鲁棒性升级 [`pose_estimator.py`]
- 改进 `refine_ellipse_from_hull()`:
  - 支持部分弧段拟合 (当近距口沿不完整)
  - 加入轴比先验约束 (基于历史观测)
  - 使用加权最小二乘 (权重 = 到边缘的距离倒数)
- 返回拟合置信度 `ellipse_fit_score`

#### B3. 自适应 ROI 扩张 [`pose_estimator.py` 或 `detection_node.py`]
- 将固定的 `RIM_MASK_DILATION` 替换为:
  ```
  dilation = max(5, min(25, bbox_width * 0.15))
  ```
- 根据 body bbox 尺寸自动调整，避免固定值导致的边界误差

#### B4. 法向平滑升级 [`pose_estimator.py`]
- 在 `PoseSmoother.update()` 中用 `slerp`（球面插值）替代线性平滑
- 公式: $\mathbf{n}_{\text{smooth}} = \text{slerp}(n_{\text{last}}, n_{\text{new}}, \alpha)$
- 避免法向在极点处的非线性扭曲

#### B5. 验收指标
- 姿态角瞬时尖峰 (|Δθ| > 5°) 次数 ↓ 50%
- 近距解算失败率 ↓ 30%
- 联合约束一致性分数 > 0.75 的观测占比 ≥ 80%

---

### **阶段D（可选，2-3天）：非线性重投影优化 [精密应用]**
**目标**: 突破轴比方法的精度上限，达到 ±1mm, ±0.2° 级别

仅在 Phase A+B 完成后、若任务需要进一步精度提升时启动。

#### D0. 适用判据
- 任务精度需求 < ±2mm 或 < ±0.5°
- 当前抖动虽已改善但仍不满足
- 可接受 15ms 的计算延迟（仅对高质量帧优化）

#### D1. 核心思想 [`pose_estimator.py::ReprojectionOptimizer`]
不依赖"轴比→倾角"直接映射，在 **6DOF 位姿空间直接优化**：
$$J(\xi) = \sum_i [\text{dist}(p_i, \text{project}(\mathbf{C}(\xi), r))]^2$$

输入轴比方法的结果作为初值，使用 Levenberg-Marquardt 非线性最小二乘收敛。

#### D2. 关键技术点
- **初值**: 轴比方法结果（好的初值保证收敛）
- **距离度量**: 混合策略（前期代数距离快速下降，后期几何距离精调）
- **异常值处理**: Huber 损失函数，抑制高光/掩膜误差的影响
- **回退机制**: 优化失败时保留轴比结果

#### D3. 预期效果
- 位姿精度：±15mm → ±1mm （3σ）
- 姿态精度：±3° → ±0.2°
- 位置抖动：8mm σ → 0.3mm σ
- 计算耗时：2ms → 15ms（可选择仅对关键帧优化）

#### D4. 集成策略
```python
if quality_score > QUALITY_MIN_REPROJECTION:
    # 仅对高质量观测进行优化
    refined_pose = reprojection_optimizer.optimize(
        edge_contours, initial_pose=轴比结果
    )
else:
    refined_pose = 轴比结果  # 低质量时回退
```

---

### **阶段C（1天）：可见性约束 + 执行级降级**
**目标**: 防近距失锁、与执行链协同

#### C1. 可见性指标定义 [`detection_node.py`]
- 新增计算函数 `assess_visibility_state(ellipse_2d, image_shape)`:
  ```python
  border_ok = (椭圆中心 to 四边最小距离) > RIM_BORDER_MARGIN_PX
  scale_ok = ellipse_major_px in [ELLIPSE_MAJOR_MIN, ELLIPSE_MAJOR_MAX]
  shape_ok = axis_ratio in [AXIS_RATIO_MIN, AXIS_RATIO_MAX]
  
  if border_ok and scale_ok and shape_ok:
      return VISIBILITY_GOOD
  elif border_ok and (scale_ok or shape_ok):
      return VISIBILITY_RISKY
  else:
      return VISIBILITY_BAD
  ```
- 在每次发布前附加 `visibility_state` 到消息中（新增自定义消息类型或标记字段）

#### C2. 执行动作映射 [`detection_node.py`]
- 在发布前根据 `visibility_state + quality_score` 生成执行建议标记:
  - `VISIBILITY_GOOD & q >= q2`: 全姿态执行
  - `VISIBILITY_GOOD & q1 <= q < q2`: 放松姿态容差
  - `VISIBILITY_RISKY`: 仅位置更新，禁止前进
  - `VISIBILITY_BAD`: 冻结目标或回退
- 标记信息写入日志或话题供执行端订阅

#### C3. 连续坏帧保护 [`detection_node.py`]
- 为每个 `track_id` 维护 `bad_visibility_count`
- 连续 `k` 帧 `VISIBILITY_BAD` → 触发警报并建议重新识别
- 参数: `BAD_VISIBILITY_THRESHOLD = 5`

#### C4. 新增参数 [`config.py`]
```python
# 可见性约束
RIM_BORDER_MARGIN_PX = 40
ELLIPSE_MAJOR_MIN_PX = 60
ELLIPSE_MAJOR_MAX_PX = 300
AXIS_RATIO_VALID_MIN = 0.15
AXIS_RATIO_VALID_MAX = 0.95

# 降级阈值
QUALITY_THRESHOLD_Q1 = 0.50
QUALITY_THRESHOLD_Q2 = 0.70
BAD_VISIBILITY_THRESHOLD = 5
```

#### C5. 验收指标
- 近距失锁率 ↓ 60%
- 执行端无"低可见性继续前冲"的危险动作
- 失败场景可安全回退

---

## 三、总体时间表

| 阶段 | 工作项 | 预计 | 依赖 | 优先级 |
|------|-------|------|------|--------|
| **A** | 质量评分 | 4h | 无 | 🔴 必做 |
| **A** | 短时融合 | 4h | 质量评分 | 🔴 必做 |
| **A** | 失锁续航 | 3h | 无 | 🔴 必做 |
| **B** | 联合约束 | 6h | A (通过质量分) | 🟡 高收益 |
| **B** | 椭圆鲁棒性 | 5h | 无 | 🟡 高收益 |
| **B** | slerp 平滑 | 2h | A | 🟡 高收益 |
| **C** | 可见性约束 | 4h | A | 🟡 防护性 |
| **C** | 执行映射 | 3h | C | 🟡 防护性 |
| **D** (可选) | 重投影优化 | 16h | A+B | 🟢 精密应用 |
| **测试验证** | 联调测试 | 8h | B+C | 🔴 必做 |

**最小化路径** (适合 ±10mm 精度需求): 
- A (1天) → B (1.5天) → [验证] → 上线

**精密应用路径** (±2mm 精度需求):
- A → B → D (2-3天) → C → [完整测试]

---

## 四、代码改动清单（优先级排序）

### 高优 (Phase A - 必做)
- [ ] `pose_estimator.py::compute_quality_score()` - 新增
- [ ] `pose_estimator.py::process_detection()` - 添加质量计算
- [ ] `detection_node.py::publish_cylinders_array()` - 质量门控
- [ ] `detection_node.py::publish_selected_target()` - 质量门控
- [ ] `detection_node.py` - 为 track_id 维护 lost_counter
- [ ] `config.py` - 添加质量/融合/续航参数
- [ ] `detection_node.py::fuse_burst_samples()` - 新增融合函数
- [ ] `detection_node.py` - 维护每个 track_id 的样本缓冲

### 中优 (Phase B - 高收益)
- [ ] `pose_estimator.py::check_body_rim_consistency()` - 新增
- [ ] `pose_estimator.py::refine_ellipse_from_hull()` - 升级鲁棒性
- [ ] `pose_estimator.py::PoseSmoother.update()` - slerp 法向平滑
- [ ] `pose_estimator.py` - 自适应 dilation 逻辑

### 中低优 (Phase C - 防护性)
- [ ] `detection_node.py::assess_visibility_state()` - 新增
- [ ] `detection_node.py` - visibility_state 追踪与发布
- [ ] 执行端订阅与动作映射 (control 侧)

### 低优 (Phase D - 可选，仅精密应用)
- [ ] `pose_estimator.py::ReprojectionOptimizer` 类 - 新增
- [ ] `pose_estimator.py::project_circle_to_image()` - 3D→2D投影
- [ ] `pose_estimator.py::point_to_ellipse_distance()` - 距离计算
- [ ] `pose_estimator.py::reprojection_optimize()` - LM优化求解
- [ ] `detection_node.py` - 集成优化器到发布流程
- [ ] `config.py` - 添加优化器参数

---

## 五、风险与回退

### 风险
1. **质量门控过严** → 可用率下降
   - 回退: 参数软化、先记日志不阻止
2. **融合延迟过大** → 响应滞后
   - 回退: 缩减 burst_sample_count，或仅在确认阶段融合
3. **Body 约束冲突** → 姿态校正不稳定
   - 回退: 权重调低，沦为参考而非强约束

### 参数调优顺序
1. 先调质量阈值 (QUALITY_MIN_PUBLISH) 找平衡点
2. 再调融合窗口大小 (BURST_SAMPLE_COUNT)
3. 最后微调各 q 子项权重

---

## 六、小结

**新版主线**: 质量评分 → 短时稳态 → 可见性约束 + 失锁续航 → (可选) 重投影优化

- **Phase A** (立即落地, 1天): 快速获得 40% 抖动改善，无重大风险。适合所有场景。
- **Phase B** (稳定后推进, 1.5天): 几何精度上限提升，近距可用性++。通用加强。
- **Phase C** (并行可选, 1天): 防护与协同，主要用于安全冗余。推荐配套。
- **Phase D** (按需启动, 2-3天): 突破轴比方法的精度上限，仅对精密应用（±2mm）必需。

此路线与现有工程流程兼容，逐步递进，可随时暂停与回退。

**决策建议**:
1. ✅ **立即启动 Phase A+B** (2.5天完成)
2. 📊 **验证性能后判断**:
   - 若满足 ±10mm 精度需求 → 启动 Phase C + 上线
   - 若需进一步改善 → 启动 Phase D (可与 Phase C 并行)
