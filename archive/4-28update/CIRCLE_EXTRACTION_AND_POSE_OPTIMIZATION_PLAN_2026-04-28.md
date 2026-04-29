# 2026-04-28 圆形提取 + 位姿估计优化计划

## 1. 当前背景

当前 `vision_detection` 主链路仍然以如下方式工作：

- `YOLO body` 负责整图中的圆柱检测与跟踪。
- `YOLO rim` 负责在 body ROI 内给出管口分割 mask。
- `pose_estimator.py` 当前仍以 `mask -> Canny -> contour -> convexHull -> fitEllipse -> 单圆几何解算` 为主。
- 下游视觉语义已经收敛为 `center + axis`，不再把视觉结果当作一个完整四元数姿态。

当前最核心的问题不是时序稳定，而是单帧 2D 几何本身不够准：

- rim 提取对反光、膨胀误差、背景边缘过于敏感。
- `convexHull + fitEllipse` 容易把错误边缘一并吞进去。
- 单圆解本身存在双解/退化问题。
- 正视场景下侧边缘消失，原有“椭圆长短轴 + 角度”方法会明显变弱。

---

## 2. 已明确的硬约束

这些约束在后续实现中应视为固定前提，不再反复讨论：

- 管口内径在比赛现场可重新测量，因此可作为已知参数。
- 外壳尺寸、缩颈尺寸、基座局部结构不作为硬尺寸先验。
- 管口和管体的深度信息不进入主估计链路。
- D435i 对白色、小尺寸、光滑目标的深度不可靠，因此 tube/rim depth 不作为先验或主修正量。
- 视觉层输出只应为：
  - `center_3d`
  - `axis_3d`
- 不再从视觉层发布“看似完整但语义不成立”的四元数姿态。

---

## 3. 总体目标

把当前方案改造成一条 **纯 RGB 主导、已知内径约束、分模式处理退化情况** 的链路：

1. 更精确地提取管口圆/椭圆。
2. 生成并验证圆口位姿候选，而不是单次拟合直接给唯一解。
3. 在可见侧边母线时，用母线做圆心修正和二义性消除。
4. 在正视或弱可观测时，进入专门的 frontal fallback，而不是硬解不可信小倾角。

---

## 4. 目标方法结构

### 4.1 圆口提取部分

这一部分重点借鉴 `Arc-support Line Segments Revisited` 的思路，但不直接移植其 MATLAB/OpenCV 2.4 实现。

应借鉴的核心思想：

- 不直接对全部边缘点做 `fitEllipse`。
- 先提取 arc-support line segments。
- 候选由局部显著弧组和成对弧组生成。
- 利用 polarity、support ratio、coverage 做严格验证。
- 候选通过后再做二次 refit。

不建议直接照搬的部分：

- 原仓库依赖 MATLAB + MEX + OpenCV 2.4.9。
- 当前项目应保持 ROS2/Python/OpenCV 现有运行环境。
- 当前项目不是通用多目标椭圆检测，而是有 YOLO body/rim ROI 的任务约束，应做任务化裁剪。

### 4.2 位姿估计部分

位姿估计改成三模式：

#### Mode A：斜视且可见侧边母线

适用条件：

- rim 椭圆可稳定提取。
- body 外轮廓中能提取到两条近似平行的侧边母线。

处理方式：

- 用已知内径的单圆几何生成两组 pose 候选。
- 从 body 轮廓中检测左右侧边直线 `l1, l2`。
- 通过两条线得到消失点 `v = l1 x l2`。
- 用 `a ~ normalize(K^-1 v)` 得到轴向方向约束。
- 用极线/极点关系修正圆心投影，不再直接使用 `fitEllipse` 的椭圆中心。
- 在两组候选解中选取与母线方向、轮廓对称性最一致的一组。

这一模式是后续精度最高的主模式。

#### Mode B：看不到稳定侧边，但椭圆仍明显偏心

适用条件：

- side lines 置信度不足。
- `b / a` 尚未退化到接近 1。

处理方式：

- 保留两组单圆候选解。
- 不使用 tube depth。
- 仅用 RGB 几何一致性评分选择候选，包括：
  - rim 边缘重投影误差
  - mask-boundary overlap
  - body 主轴一致性
  - 圆心是否落在 body 轮廓中心线附近

这一模式是无侧边时的常规兜底模式。

#### Mode C：正视退化 / 近圆模式

适用条件：

- `b / a` 非常接近 1。
- side lines 不可见或极弱。

处理方式：

- 不再硬解不可信的小倾角。
- 优先做 circle-specific fitting 和验证。
- `center_2d` 由 circle/near-circle 拟合给出。
- `Z` 由已知内径的单圆有尺度解给出。
- `axis` 只取接近相机光轴的方向，并明确打上 `frontal / low_observability` 标记。

这一模式强调“保守但可信”，而不是“强行解出完整姿态”。

---

## 5. 具体实现计划

### Phase 1：重写 rim 提取入口

目标：

- 把当前 `convexHull -> fitEllipse` 主链路替换为 arc-support inspired pipeline。

建议改动点：

- 文件：
  - `src/vision_detection/vision_detection/pose_estimator.py`
  - `src/vision_detection/vision_detection/detection_node.py`
- 当前入口：
  - `RGBCylinderPoseEstimator.process_detection(...)`

实现方向：

- 保留 `YOLO rim mask` 作为搜索范围。
- 不再对膨胀后的整块 mask 直接做几何拟合。
- 从 rim mask 生成一个窄边界带 `boundary band`。
- 在 band 内提取边缘和短线段。
- 把线段分成：
  - 候选弧段
  - 候选长直线段

建议新增函数：

- `extract_rim_boundary_band(...)`
- `detect_arc_support_segments(...)`
- `detect_body_side_lines(...)`

### Phase 2：椭圆/圆候选生成与验证

目标：

- 不再“一次拟合直接出最终椭圆”，而是先生成多个候选并筛选。

实现方向：

- 用弧段组合生成候选 circle/ellipse。
- 对每个候选计算：
  - support inlier ratio
  - angular coverage
  - polarity consistency
  - body consistency
  - mask overlap
- 对高分候选做二次 refit。

建议新增函数：

- `generate_candidate_ellipses(...)`
- `score_candidate_ellipse(...)`
- `refit_candidate_ellipse(...)`
- `fit_circle_or_ellipse_fallback(...)`

### Phase 3：已知内径单圆 pose 候选解

目标：

- 明确产出“两组候选解”，不再假装单圆总能直接给唯一真解。

实现方向：

- 基于已知内径和相机内参，从圆/椭圆候选求单圆 pose candidates。
- 输出：
  - `candidate_0`
  - `candidate_1`
- 每个候选包含：
  - `center_3d`
  - `axis_3d`
  - `reprojection metrics`

建议新增函数：

- `solve_circle_pose_candidates(...)`

### Phase 4：侧边母线分支与圆心修正

目标：

- 在斜视场景下利用圆柱母线进一步提精度并消二义性。

实现方向：

- 从 body ROI 或 body mask 边界中检测两条主要母线。
- 利用母线求轴向消失点。
- 在 side-line 置信度足够高时：
  - 修正圆心投影
  - 修正轴向
  - 从两组 pose candidates 中选真解

建议新增函数：

- `estimate_axis_from_side_lines(...)`
- `correct_projected_circle_center(...)`
- `disambiguate_with_side_lines(...)`

### Phase 5：frontal fallback

目标：

- 专门处理“看不到侧边、椭圆近似正圆”的退化场景。

实现方向：

- 当 `axis_ratio >= threshold_front` 时，不沿用 oblique 模式。
- 走单独的 circle mode。
- 输出额外状态：
  - `observability_state = frontal`
  - `pose_confidence`

建议新增函数：

- `frontal_fallback(...)`

### Phase 6：环境平面只做弱辅助

目标：

- 不把木质基座和平面结构当主解算对象，但保留以后作为弱约束的可能。

当前结论：

- 这些平面不应在第一轮实现中写成硬约束。
- 若后续接入，也只应做：
  - 候选解几何一致性筛选
  - 场景主方向辅助
- 不应用其深度去修 tube pose 主链路。

---

## 6. 建议的代码结构调整

建议在 `pose_estimator.py` 中逐步形成如下模块：

- `extract_rim_boundary_band`
- `detect_arc_support_segments`
- `generate_and_score_candidates`
- `fit_circle_or_ellipse_fallback`
- `solve_circle_pose_candidates`
- `detect_body_side_lines`
- `score_candidates_rgb_only`
- `disambiguate_with_side_lines`
- `frontal_fallback`

建议保留一个主入口：

- `process_detection(...)`

但把内部逻辑由“单通道直线流程”改成“分模式调度器”。

---

## 7. 第一轮实现的明确边界

为了避免一次改太多，建议第一轮只做下面这些事情：

1. 删除当前 `convexHull -> fitEllipse` 的主导地位。
2. 把 rim 几何提取改成 boundary band + arc-support inspired candidate pipeline。
3. 先实现 `circle/ellipse candidate -> score -> refit`。
4. 先把 `single circle -> two pose candidates` 结构搭起来。
5. 暂时不把环境平面纳入主链路。
6. 暂时不重新引入 tube/rim depth。

第二轮再继续做：

1. side-line 消失点与圆心修正。
2. frontal fallback 的细化。
3. 调参与数据集验证。

---

## 8. 验证与调试建议

第一轮实现完成后，建议至少输出以下调试信息：

- rim boundary band 可视化
- arc-support segments 可视化
- ellipse/circle candidates 可视化
- 最终采用候选与被拒候选的分数
- `mode = oblique / weak_side / frontal`
- `axis_ratio`
- `support ratio`
- `coverage`
- `candidate selection reason`

建议的评估指标：

- 圆心 2D 重投影稳定性
- 静态重复测量的 `center_3d` 标准差
- `axis_3d` 角度抖动
- 正视场景成功率
- 反光/局部遮挡场景下的鲁棒性

---

## 9. 当前最优先的执行顺序

推荐优先级如下：

1. rim 提取重构
2. candidate generation + scoring
3. known-circle pose candidates
4. side-line disambiguation
5. frontal fallback

这条顺序最符合当前问题的根因，也最容易让后续 agent 逐步实现和验证。
