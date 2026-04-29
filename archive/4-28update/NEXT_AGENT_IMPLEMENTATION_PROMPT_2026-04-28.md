# 给下一位 Agent 的 Prompt

请基于当前仓库 `/home/arnoyin/agx_arm_ws`，继续实现 `vision_detection` 中“圆形提取 + 位姿估计”的优化。先阅读以下文件：

- `archive/4-28update/CIRCLE_EXTRACTION_AND_POSE_OPTIMIZATION_PLAN_2026-04-28.md`
- `archive/4-28update/LITERATURE_AND_PROJECT_SURVEY_2026-04-28.md`
- `src/vision_detection/vision_detection/pose_estimator.py`
- `src/vision_detection/vision_detection/detection_node.py`

## 当前背景

当前项目已经完成两件事情：

1. 视觉输出语义已经改为 `center + axis`，不再依赖视觉四元数。
2. 原有多层时序稳定策略已经删除或弱化，当前重点转到“单帧几何本身做准”。

但是 `pose_estimator.py` 里仍然保留着旧的 rim 提取主链路：

- `mask -> Canny -> contour -> convexHull -> fitEllipse -> 单圆几何`

这条链路需要开始重构。

## 硬约束

请把下面这些约束当成固定前提：

- 管口内径是已知参数，可在比赛现场重新测量。
- 外壳尺寸、缩颈尺寸、木质基座尺寸都不作为硬尺寸先验。
- 不要把 tube/rim depth 纳入主估计链路。
- D435i 对白色、小尺寸、光滑目标的深度不可靠，因此不要重新引入 tube depth 先验。
- 视觉层目标输出只有：
  - `center_3d`
  - `axis_3d`
- 不要重新把视觉结果包装成“伪完整四元数姿态”。

## 参考方向

本轮实现主要参考：

- Arc-support Line Segments Revisited: An Efficient and High-quality Ellipse Detection
- Circle detection by arc-support line segments
- single-circle pose estimation / circle + lines 消歧义相关论文

重点不是把论文完整复刻，而是把它们的思路迁移到当前 YOLO ROI 任务上。

## 第一阶段任务

请优先完成“圆口提取前端”的第一轮重构，不要求一次把全部几何都做完。

### 第一阶段目标

把当前 `convexHull -> fitEllipse` 的主导地位替换掉，形成一个更可靠的 rim candidate pipeline。

### 建议交付范围

1. 在 `pose_estimator.py` 中新增以下模块或等价模块：
   - `extract_rim_boundary_band`
   - `detect_arc_support_segments`
   - `generate_and_score_candidates`
   - `fit_circle_or_ellipse_fallback`

2. 保持 `process_detection(...)` 作为外部主入口，但把内部流程改成：
   - 从 `rim mask` 生成窄边界带，而不是直接对整块膨胀 mask 拟合。
   - 从边界带中提取弧段/短线段。
   - 生成少量 circle/ellipse candidates。
   - 用 support ratio、coverage、mask overlap 等评分选最优候选。
   - 对最优候选做 refit。

3. 为后续几何消歧义预留接口，但第一阶段可以先不完整实现：
   - `solve_circle_pose_candidates`
   - `detect_body_side_lines`
   - `frontal_fallback`

4. 输出更多调试信息，至少包括：
   - rim boundary band 可视化
   - arc-support / candidate 可视化
   - 每个候选的评分
   - 最终选择理由

## 第二阶段任务

如果第一阶段完成并且结构清楚，再继续做：

1. 已知内径下的 `single circle -> two pose candidates`
2. side-line 分支：
   - 从 body 轮廓提取侧边母线
   - 用 side lines 做 axis 约束和圆心修正
3. frontal mode：
   - 当 `axis_ratio` 接近 1 时切到 near-circle / frontal fallback

## 不要做的事情

- 不要重新引入 tube/rim depth 融合。
- 不要重新加多层时序平滑来掩盖 2D 几何误差。
- 不要直接把 MATLAB/MEX 仓库原样抄进来。
- 不要为了“先跑起来”继续依赖 `convexHull` 作为最终几何主依据。

## 你应该如何工作

请直接在当前代码上实现第一阶段改动，而不是只给分析。

实现时请：

- 先检查当前 `pose_estimator.py` 和 `detection_node.py` 的入口关系。
- 尽量保持现有 ROS2 话题接口不变。
- 保持输出仍为 `center + axis` 语义。
- 做完后运行最基本的静态检查或可执行性检查。
- 最终明确说明：
  - 改了哪些文件
  - 当前完成到哪个阶段
  - 还剩哪些后续步骤

## 补充说明

如果你认为第一阶段里“arc-support segments 全量实现”过大，可以先做一个任务化裁剪版：

- 不做通用全图 ellipse detector
- 只在 YOLO rim ROI 内做 boundary band + segment grouping + candidate verification

这样更适合当前仓库，也更容易快速验证效果。
