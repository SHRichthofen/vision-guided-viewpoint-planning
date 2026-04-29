# D435i 深度先验融合计划（2026-04-20）

## 1. 背景与目标
当前管口位姿以 RGB 椭圆几何解算为主，在近似正圆（轴比接近 1）场景下距离与姿态易退化。

目标：
1. 从 Stage1（body）分割 mask 统计 D435i 深度先验；
2. 将先验用于 `Z` 融合，降低距离抖动与偏差；
3. 在近似正圆时提高深度先验权重，抑制几何退化。

---

## 2. 数据链路
1. RealSense 同时启动 `color + depth`；
2. 使用 `align(color)` 对齐 depth 到 color 像素坐标；
3. 对每个 tube 的 body mask 统计 `depth_prior_m` 与置信度；
4. 传递到 `pose_estimator.process_detection()`；
5. 与椭圆深度 `Z_ellipse` 融合得到 `Z_fused`。

融合形式：
\[
Z_{fused} = w \cdot Z_{prior} + (1-w) \cdot Z_{ellipse}
\]
其中 \(w\) 由深度置信度和退化状态共同决定。

---

## 3. 模块设计

### 3.1 vision_detection/detection_node.py
- 启动 depth stream（z16, 640x480@30）并读取 depth scale；
- `align(color)` 获取对齐后的深度图（米）；
- 新增 `compute_depth_prior_from_body_mask()`：
  - 有效范围过滤（min/max）；
  - 百分位截断（10%-90%）；
  - 中位数 + IQR + valid_ratio；
  - 输出先验深度与置信度；
- 对每个 track 做深度先验时序平滑（EMA）；
- 将 `depth_prior_m/confidence` 传给 pose estimator；
- 在选中目标日志中附加 `z_ellipse / z_prior / z_fused / weight`。

### 3.2 vision_detection/pose_estimator.py
- 扩展 `process_detection()` 参数：
  - `depth_prior_m=None`
  - `depth_prior_confidence=0.0`
- 在椭圆几何深度后执行融合：
  - 常规按置信度给权重；
  - 近似正圆（axis_ratio > threshold）提升先验权重；
  - 先验与几何差异过大时降权；
- 返回附加调试字段：
  - `z_ellipse`, `z_prior`, `z_fused`, `z_fusion_weight`, `depth_prior_confidence`, `near_circle_degenerate`。

### 3.3 vision_arm_control/vision_to_arm_transform.py
- `camera_link -> camera_color_optical_frame` 改为“优先读取运行时 TF”；
- 启动时短时等待 D435i TF，读取成功则不发布固定静态值；
- 读取失败再回退到参数中的硬编码值（兼容离线/仿真）。

---

## 4. 参数建议
- 深度有效范围：0.15m ~ 1.20m；
- 有效率下限：0.20；
- IQR 上限：0.05m；
- 融合权重：min 0.15 / max 0.70；
- 近圆阈值：axis_ratio > 0.93；
- 近圆权重下限：0.65；
- 先验冲突阈值：|Z_prior - Z_ellipse| > 0.20m 时降权。

---

## 5. 验证计划
1. 固定目标点，记录 100 帧：
   - 比较 `Z_ellipse` 与 `Z_fused` 标准差；
   - 关注 base 系 y 偏差稳定性；
2. 近圆场景与倾斜场景分别测试；
3. 查看选中目标日志，确认融合权重与质量分数符合预期。

验收建议：
- `Z` 抖动下降 >= 30%；
- 近圆场景不再出现明显深度跳变。

---

## 6. 风险与回退
- 反光/孔洞导致 depth 无效：启用 valid_ratio 门槛；
- TF 未及时发布：回退到静态参数；
- 先验误导：保留冲突降权与质量门控。
