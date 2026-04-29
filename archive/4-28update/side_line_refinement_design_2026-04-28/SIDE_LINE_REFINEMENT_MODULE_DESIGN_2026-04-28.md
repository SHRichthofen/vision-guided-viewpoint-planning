# 2026-04-28 侧边精修模块设计思路

## 1. 设计目标

本模块的目标不是“直接从第一层圆柱 `body mask` 抄出两条边”，而是：

- 用第一层 `body mask` 提供圆柱体的空间范围与粗主方向
- 回到原始 RGB 图像中，在 `body mask` 附近提取相对精确的左右侧边
- 为后续 `side-line disambiguation / projected center correction / axis refinement` 提供高质量输入

换句话说，`body mask` 是搜索先验，不是最终侧边。

## 2. 必须遵守的总体计划

请始终服从 `archive/4-28update/CIRCLE_EXTRACTION_AND_POSE_OPTIMIZATION_PLAN_2026-04-28.md` 的总体路线：

1. 继续保持纯 RGB 主导
2. 不把 tube/rim depth 重新引回主估计链路
3. 视觉输出仍然只保留：
   - `center_3d`
   - `axis_3d`
4. 不重新把结果包装成伪完整四元数姿态
5. 不靠额外时序平滑掩盖单帧几何问题

## 3. 当前已完成进度

截至当前仓库状态，已经完成：

- rim 前端从 `convexHull -> fitEllipse` 切换到 `boundary band + candidate + refit`
- `single circle -> pose candidates` 的第一版结构
- `RGB-only` 的候选位姿选解
- `frontal / weak_side` 模式标记
- body-mask 侧边检测的第一版占位实现与可视化接入

相关记录见：

- `archive/4-28update/IMPLEMENTATION_LOG_2026-04-28.md`

## 4. 当前已知问题

当前验证已经明确暴露以下问题：

- 边缘提取仍有抖动，导致位姿有角度抖动
- `frontal` 模式下，轴向候选会来回跳变
- `weak_side` 模式在大倾角场景下，红色 mask 与边缘提取不重合
- 当前“侧边模式”几乎无法被激活
- 当前 ROI 可视化对侧边的显示不够清楚

这些问题已经记录在实现日志中，下一步应优先解决“侧边输入质量”而不是直接上最终消歧。

## 5. 为什么当前侧边实现不够

当前版本的 `detect_body_side_lines(...)` 存在两个根本问题：

1. `body_dir` 仍然主要来自 axis-aligned `body_bbox`
2. 侧边直接在 `body mask` 外轮廓上拟合，没有回到 RGB 中做边缘精修

这意味着：

- 对任意斜角目标，主方向估计不准
- `mask` 锯齿、外扩、漏分会直接污染侧边
- 虽然“看起来像有侧边”，代码仍然可能检测不出来

## 6. 推荐的新模块结构

建议把侧边模块做成下面这条链路：

### Step A：body mask 主轴估计

输入：

- `body mask`

建议做法：

- 对最大连通域轮廓做 PCA
或
- 使用 `cv2.minAreaRect`

输出：

- `body_center_2d`
- `body_axis_dir_2d`
- `body_normal_dir_2d`
- `body_long_span`
- `body_short_span`

要求：

- 不再用“水平/竖直二选一”的 bbox 主方向
- 主方向必须允许任意图像角度

### Step B：左右搜索带生成

输入：

- `body mask`
- `body_axis_dir_2d`
- `body_normal_dir_2d`

做法：

- 沿 `body_normal_dir_2d` 在左右两侧建立窄搜索带
- 搜索带要覆盖圆柱长度方向的大部分区域
- 搜索带厚度应只覆盖侧边附近，不包含大量内部区域

建议输出：

- `left_band_mask`
- `right_band_mask`
- `left_seed_points`
- `right_seed_points`

### Step C：RGB 边缘精修

输入：

- 原始 `roi_img`
- 左右搜索带

做法：

- 在 band 内回到原图找真正的亮度边缘，而不是直接使用 `body mask` 边界
- 推荐使用：
  - Sobel / Scharr 梯度
  - 局部 Canny
  - 沿 `body_normal_dir_2d` 的一维扫描找梯度极值

核心要求：

- 侧边支持点应尽量贴真实图像边界
- 不要让顶部弧边、底部弧边、背景纹理大面积混入

建议输出：

- `left_edge_points_refined`
- `right_edge_points_refined`

### Step D：鲁棒直线拟合

输入：

- 左右精修边缘点

做法：

- 各自拟合一条线
- 可优先用 `cv2.fitLine`
- 如果离群点仍多，可引入简化 RANSAC

建议输出：

- `side_line_left`
- `side_line_right`

每条线至少要附带：

- `p0`
- `p1`
- `direction`
- `residual_px`
- `support_count`
- `span_ratio`
- `body_alignment`

### Step E：双侧一致性验证

只有当左右两条线同时满足以下条件时，才允许进入后续 side-line 模式：

- 与 body 主轴基本平行
- 各自长度足够
- 拟合残差足够小
- 左右分离足够大
- 两条线大致互相平行
- 支持点分布覆盖圆柱长度方向的足够比例

建议额外增加：

- `parallel_score`
- `separation_score`
- `support_balance_score`

### Step F：调试可视化

要让调试图能回答“为什么失败”，而不仅是“画了/没画”。

建议同时支持两种视图：

1. `result view`
   - 只显示最终采用的 `S0/S1`
   - 显示关键分数

2. `diagnostic view`
   - 显示左右搜索带
   - 显示左右支持点
   - 显示候选线
   - 明确显示失败原因

建议至少输出：

- `body_axis_dir`
- `left/right band`
- `left/right refined points`
- `selected side lines`
- `reject reason`

## 7. 与现有 pose 链路的衔接顺序

推荐按下面顺序推进，而不是一步到位：

### Phase 1：先把侧边检测本身做稳

本阶段目标：

- 任意合理斜角下能够稳定提取 `S0/S1`
- 可视化能清楚表明“用了哪两条线”
- 可视化能解释“为什么没有激活侧边模式”

此阶段先不要急着把侧边强行接进最终 pose 修正。

### Phase 2：把侧边作为模式切换依据

当 `S0/S1` 稳定后，再把：

- `observability_state = oblique`

建立在“侧边检测真的可信”之上，而不是只看是否找到了两条线。

### Phase 3：侧边参与位姿消歧

后续再加入：

- `estimate_axis_from_side_lines(...)`
- `disambiguate_with_side_lines(...)`
- `correct_projected_circle_center(...)`

## 8. 当前实现建议

推荐直接替换当前这版“mask 极值点 + fitLine”的粗方法，不要继续在它上面硬调阈值。

优先修改：

- `src/vision_detection/vision_detection/pose_estimator.py`
- `src/vision_detection/vision_detection/config.py`

必要时微调：

- `src/vision_detection/vision_detection/detection_node.py`

## 9. 测试通过标准

第一阶段侧边精修模块的通过标准建议是：

1. 对明显斜视的圆柱，`S0/S1` 能稳定显示
2. `S0/S1` 大致贴真实图像侧边，而不是贴粗糙 mask 外沿
3. 固定视角下，`S0/S1` 不应明显跳边
4. 调试图能明确区分：
   - 没找到支持点
   - 找到点但拟合失败
   - 拟合成功但一致性未过阈值
5. 只有在 `S0/S1` 质量足够高时才允许切入 `oblique`

## 10. 修改前备份要求

后续任何 agent 在继续改代码前，必须先备份当前代码。

建议固定流程：

1. 在 `archive/4-28update/` 下新建新的备份目录
2. 备份至少以下文件：
   - `src/vision_detection/vision_detection/pose_estimator.py`
   - `src/vision_detection/vision_detection/detection_node.py`
   - `src/vision_detection/vision_detection/config.py`
3. 只有备份完成后再进行修改

这条要求应视为硬约束。
