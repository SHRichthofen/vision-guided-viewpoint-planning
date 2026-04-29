# 2026-04-28 第一阶段实现记录

## 1. 任务目标

本轮目标是把 `vision_detection/pose_estimator.py` 中原来的：

- `mask -> Canny -> contour -> convexHull -> fitEllipse -> 单次几何解`

改成第一阶段可落地版本：

- `rim mask -> boundary band -> arc/line segments -> candidate generation + scoring -> refit`

同时保持：

- 外部 ROS2 入口仍然是 `process_detection(...)`
- 视觉输出仍然是 `center_3d + axis_3d`
- 不重新把深度先验放回主估计链路

## 2. 备份要求落实

在正式修改前，已先保留当前源码副本：

- `archive/4-28update/code_backup_2026-04-28/src/vision_detection/vision_detection/pose_estimator.py`
- `archive/4-28update/code_backup_2026-04-28/src/vision_detection/vision_detection/detection_node.py`
- `archive/4-28update/code_backup_2026-04-28/src/vision_detection/vision_detection/config.py`

## 3. 修改步骤

### Step 1: 扩展配置项

文件：

- `src/vision_detection/vision_detection/config.py`

新增或调整：

- `RIM_MASK_CLOSE_*`
- `RIM_BOUNDARY_BAND_*`
- `ARC_SEGMENT_*`
- `CANDIDATE_*`
- `HOUGH_*`
- `DEBUG_TOP_CANDIDATES`
- `ENABLE_DEPTH_PRIOR = False`

目的：

- 让 rim mask 先做轻量闭运算，不再用大膨胀直接主导几何。
- 给第一阶段 candidate pipeline 提供独立阈值。
- 默认把深度先验降到主链路之外。

### Step 2: 重写圆口提取主链路

文件：

- `src/vision_detection/vision_detection/pose_estimator.py`

新增模块：

- `extract_rim_boundary_band`
- `detect_arc_support_segments`
- `generate_and_score_candidates`
- `fit_circle_or_ellipse_fallback`
- `refit_candidate_ellipse`

为第二阶段预留的接口：

- `detect_body_side_lines`
- `solve_circle_pose_candidates`
- `frontal_fallback`

实际变化：

- 不再使用 `convexHull` 作为主导几何。
- 先从 rim mask 生成窄 `boundary band`。
- 只在 `boundary band` 内做边缘和弧段提取。
- 从弧段单体、弧段配对、弧段并集、mask boundary 生成少量候选。
- 使用 `support_ratio + coverage + mask_overlap + residual` 评分。
- 对最优候选做一次 inlier refit。
- 保留 `center + axis` 输出语义，但新增：
  - `rim_candidates`
  - `selected_candidate_reason`
  - `rim_pipeline_stage`
  - `observability_state`
  - `pose_candidates_3d`（第一阶段占位）

### Step 3: 调整 detection node 的 mask 预处理和调试透传

文件：

- `src/vision_detection/vision_detection/detection_node.py`

调整：

- `run_rim_pipeline()` 中不再把 rim mask 做大膨胀后直接送拟合。
- 改为对 rim mask 先做轻量闭运算，再交给 `pose_estimator` 提取 `boundary band`。
- 在 semantics payload 中增加可选调试字段：
  - `rim_pipeline_stage`
  - `selected_candidate_reason`
  - `rim_candidates`
  - `observability_state`
  - `pose_candidates_3d`
  - `depth_prior_ignored`
  - `side_line_count`

## 4. 当前完成度

当前已经完成：

- 第一阶段前端结构重构
- 候选生成与评分
- 最优候选 refit
- 调试可视化和调试字段透传
- 深度先验退出主链路
- 基于当前稳定椭圆的 `single circle -> pose candidates` 第一版结构
- `RGB-only` 双候选选解
- `frontal / weak_side` 观测模式标记

## 4.1 第二轮位姿估计增量

在继续修改前，已新增一份阶段性备份：

- `archive/4-28update/code_backup_phase2_2026-04-28/src/vision_detection/vision_detection/pose_estimator.py`
- `archive/4-28update/code_backup_phase2_2026-04-28/src/vision_detection/vision_detection/detection_node.py`
- `archive/4-28update/code_backup_phase2_2026-04-28/src/vision_detection/vision_detection/config.py`

本轮位姿估计增量主要包括：

1. 在 `pose_estimator.py` 中把原先“单椭圆直接出单解”的后端改成：
   - `solve_circle_pose_candidates(...)`
   - `score_candidates_rgb_only(...)`
   - `selected_pose_candidate_id / selected_pose_reason`

2. 对同一条稳定椭圆生成：
   - `weak_side` 模式下的两组候选
   - `frontal` 模式下的单候选保守解

3. 候选选解当前使用的 RGB 几何项：
   - `body_center_score`
   - `centerline_score`
   - `body_axis_score`

4. 调试可视化新增：
   - `mode=frontal/weak_side`
   - `pose candidate score`
   - `selected pose reason`

5. 语义 payload 新增：
   - `selected_pose_candidate_id`
   - `selected_pose_reason`
   - `pose_candidates_3d`

## 4.2 ROI 调试图收敛

为避免 ROI 调试图被边缘提取信息淹没，又新增一份界面调整前备份：

- `archive/4-28update/code_backup_debug_ui_2026-04-28/src/vision_detection/vision_detection/pose_estimator.py`
- `archive/4-28update/code_backup_debug_ui_2026-04-28/src/vision_detection/vision_detection/detection_node.py`
- `archive/4-28update/code_backup_debug_ui_2026-04-28/src/vision_detection/vision_detection/config.py`

当前默认改为：

- 隐藏 `boundary band / edges / arc segments / line segments` 这些前端提取层
- ROI 图上只保留最终椭圆、少量备选候选和中心标记
- 位姿信息改为简洁卡片，重点显示：
  - `mode`
  - `selected pose candidate`
  - `quality / fit / consistency`
  - `P0/P1 score`
  - `ctr / mid / ax`

如果后续需要重新查看前端提取细节，可把：

- `ROI_DEBUG_SHOW_EXTRACTION_LAYERS`

改回 `True`。

## 4.3 当前已观察到的问题

本轮视频流验证中，已经明确记录以下缺陷：

- 边缘提取仍存在一定抖动，导致最终位姿有小幅角度抖动。
- `frontal` 模式下，轴线会在两种选择之间来回跳变。
- `weak_side` 模式在大倾角场景下，红色 mask 与边缘提取结果仍可能不重合。

当前判断：

- 这些问题暂时不通过时序平滑掩盖。
- 下一步优先继续按计划引入 `side-line` 线索，作为后续消歧与圆心修正的补足方向。

## 4.4 侧边检查接入

在继续修改前，已新增一份侧边分支接入前的备份：

- `archive/4-28update/code_backup_side_lines_2026-04-28/src/vision_detection/vision_detection/pose_estimator.py`
- `archive/4-28update/code_backup_side_lines_2026-04-28/src/vision_detection/vision_detection/detection_node.py`
- `archive/4-28update/code_backup_side_lines_2026-04-28/src/vision_detection/vision_detection/config.py`

本轮新增：

- 将 `body mask` 正式传入 `pose_estimator`
- 从 `body mask` 轮廓极值点中提取两条候选侧边
- 在 ROI 调试图中高亮并标注当前实际使用的两条侧边：
  - `S0`
  - `S1`
- 在语义 payload 中增加：
  - `side_lines_2d`

当前阶段说明：

- 侧边线已经进入“检测与可视化检查”阶段
- 还没有完整参与最终 pose 消歧和圆心修正
- 下一步可以继续接：
  - `disambiguate_with_side_lines`
  - `correct_projected_circle_center`

当前还没有完成：

- 已知内径下的单圆双解完整求解
- side-line 分支的真实几何消歧义
- near-circle / frontal mode 的专门几何解

## 4.5 侧边精修阶段推进（本轮）

在阅读最新 `side_line_refinement` 设计要求后，本轮先明确阶段目标与工作原则，再推进代码修改。

### 当前任务目标

本轮目标仍然是：

- 先把“侧边检测本身”做稳
- 让 ROI 调试图能解释“为什么没有进入侧边模式”
- 只有在侧边质量真的可靠时，才允许 `observability_state = oblique`

本轮**还没有**进入：

- `disambiguate_with_side_lines(...)`
- `correct_projected_circle_center(...)`
- side-line 真正参与最终 pose 消歧

也就是说，本轮阶段性质明确为：

- **侧边检测与可视化增强**

而不是：

- **side-line 已参与 pose 消歧**

### 本轮坚持的工作原则

- 继续保持纯 RGB 主导
- `body mask` 只作为搜索约束，不直接当最终侧边
- 不再依赖 axis-aligned bbox 的水平/竖直方向代表真实 body 主轴
- 不靠放宽阈值假装激活 `oblique`
- 不靠时序平滑掩盖单帧几何问题
- 输出语义仍然只保留 `center_3d + axis_3d`

### 本轮代码增量

本轮已在 `pose_estimator.py` 中推进：

1. `body mask -> PCA 主轴估计`
2. `左右窄搜索带` 生成
3. `回到 RGB` 做法向梯度精修
4. `左右侧边` 支持点聚合与鲁棒直线拟合
5. `双侧一致性` 评分与 reject reason 输出
6. 更严格的 `oblique` 门控

同时新增：

- ROI 中左右搜索带显示
- 左右支持点显示
- 候选侧边线显示
- `side_line_status / side_line_reject_reason / side_line_pair_metrics`

### 当前阶段结论

截至本轮，代码语义应理解为：

- 已进入“侧边检测质量提升 + 可视化解释增强”阶段
- 还没有进入“side-line 真正参与 pose 消歧”的阶段

## 4.6 侧边逻辑回退与重整（本轮）

在进一步联调后，确认上一版侧边精修实现存在两个明显问题：

- 代码体量膨胀过快，可读性和可调试性明显下降
- 侧边检测被错误地绑进了第二层 rim ROI 的局部视野，而不是第一层 YOLO body mask

因此本轮做了谨慎回退，并重新明确实现逻辑：

### 当前采用的新逻辑

1. 第一层 YOLO 的 `body mask` 是侧边检测的唯一输入来源
2. 第二层 YOLO 的 `rim mask` 仍只负责管口圆/椭圆提取
3. 侧边检测不再在 rim ROI 的局部小图里做复杂精修
4. 侧边检测改为：
   - 基于 `body mask` 做主轴估计
   - 在 `body mask` 轮廓上取左右两侧极值点
   - 分别拟合两条侧边线
   - 用残差、覆盖比例、平行性、分离度做轻量门控
5. 只有 body-mask 侧边通过门控时，才允许切入 `oblique`

### 这次回退的目的

- 先恢复代码可读性
- 先恢复“我能看懂、能定位、能 debug”的实现结构
- 先保证侧边来源正确，再考虑后续是否需要更强的 RGB 精修

### 当前阶段说明

截至此版：

- 已回退掉上一轮过重的侧边精修结构
- 已重新把侧边来源绑定到第一层 `body mask`
- 还没有进入 side-line 深度参与 pose 消歧的阶段

## 5. 基本检查

已执行：

```bash
python3 -m py_compile src/vision_detection/vision_detection/config.py src/vision_detection/vision_detection/pose_estimator.py src/vision_detection/vision_detection/detection_node.py
```

结果：

- 通过

## 6. 后续建议

下一轮建议按下面顺序继续：

1. 先采几张当前比赛场景图，观察 `debug_roi` 里 `boundary band / candidate` 是否稳定贴 rim。
2. 如果 candidate 还容易偏，优先调 `ARC_SEGMENT_*` 和 `CANDIDATE_*` 阈值。
3. 等 rim candidate 稳定后，再实现 `solve_circle_pose_candidates()` 的双解结构。
4. 最后接 `side lines` 做 axis 约束和圆心修正。
