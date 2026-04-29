# 给下一位 Agent 的 Prompt（更新版）

请基于当前仓库 `/home/arnoyin/agx_arm_ws`，继续推进 `vision_detection` 中“侧边稳定化 + 后续位姿消歧”的下一步工作。

开始前请先阅读：

- `archive/4-28update/CIRCLE_EXTRACTION_AND_POSE_OPTIMIZATION_PLAN_2026-04-28.md`
- `archive/4-28update/IMPLEMENTATION_LOG_2026-04-28.md`
- `archive/4-28update/side_line_refinement_design_2026-04-28/SIDE_LINE_REFINEMENT_MODULE_DESIGN_2026-04-28.md`
- `archive/4-28update/side_line_refinement_design_2026-04-28/NEXT_AGENT_SIDE_LINE_REFINEMENT_PROMPT_2026-04-28.md`
- `src/vision_detection/vision_detection/pose_estimator.py`
- `src/vision_detection/vision_detection/detection_node.py`
- `src/vision_detection/vision_detection/config.py`

## 先执行的硬要求

在做任何代码修改前，必须先备份当前代码。

请先在 `archive/4-28update/` 下创建新的备份目录，例如：

- `archive/4-28update/code_backup_side_refine_<date-or-step>/`

并至少备份：

- `src/vision_detection/vision_detection/pose_estimator.py`
- `src/vision_detection/vision_detection/detection_node.py`
- `src/vision_detection/vision_detection/config.py`

这是硬要求，不要跳过。

## 当前必须遵守的工作准则

请严格遵守下面这些原则，这些是当前阶段已经明确收敛下来的约束：

1. 继续保持纯 RGB 主导
2. 不把 tube/rim depth 重新引入主估计链路
3. 视觉输出仍然只保留：
   - `center_3d`
   - `axis_3d`
4. 不要靠时序平滑掩盖单帧几何问题
5. 不要把视觉结果重新包装成伪完整四元数姿态
6. 不要把代码重新膨胀成难以检查和 debug 的大块复杂结构
7. 第一层 YOLO 的 `body mask` 是侧边检测的唯一输入来源
8. 第二层 YOLO 的 `rim mask` 仍然只负责管口圆/椭圆提取

特别强调：

- 当前阶段优先级是“能看懂、能定位、能 debug”
- 如果要增强几何逻辑，优先做小步、可解释、可回退的修改
- 不要为了激活 `oblique` 而堆很多难维护的搜索带/评分结构

## 当前实现状态

截至当前仓库状态，下面这些已经完成：

- rim 前端已重构为 `boundary band + candidate + refit`
- `single circle -> pose candidates` 第一版已接入
- `RGB-only` 候选选解已接入
- `frontal / weak_side / oblique` 观测模式标记已接入
- 第一层 YOLO 的 `body mask` 已正式传入 `pose_estimator`
- 当前侧边检测已改为基于第一层 `body mask` 做主轴估计与两侧轮廓拟合
- 通过参数调整，当前已经能够输出两条侧边
- 右侧调试窗口已扩展到第一层 `body mask` 的整段圆柱范围
- 调试窗口中已保留：
  - 整根圆柱的 `body mask` 轮廓
  - 当前 side lines
  - rim 椭圆调试结果

## 当前已经确认的事实

请把下面这些结论视为这轮工作的直接输入，不要重复走回头路：

1. 第一层 `cylinder_best.pt` 不是纯 bbox 检测模型，而是分割模型
2. 第一层 YOLO 当前实际给出了：
   - `bbox_xywh`
   - `raw_body_mask`
3. 当前侧边拟合的真正输入已经是第一层 `body mask`
4. 当前不稳定的主要原因不是“没有侧边”，而是：
   - `body mask` 提得离真实边缘太近
   - `mask` 外轮廓有抖动、弧边和端部过渡
   - 导致 side line 有时通过、有时卡在 `line_quality_failed`
5. 当前跳变现象表现为：
   - `observability_state` 在 `oblique` 和 `weak_side` 之间来回切换

## 当前最关键的问题

虽然当前通过参数调整已经可以提取出两条边，但还没有达到“稳定可用”的程度。

当前最主要的问题是：

- side line 会随着 `body mask` 轮廓抖动而抖动
- 某些帧里虽然能拟合出线，但质量门控会掉回 `weak_side`
- 当前跳变的直接表现不是“完全没线”，而是：
  - 一会儿线通过门控
  - 一会儿 `span_ratio / residual / alignment` 之类的条件不过

换句话说，当前已经从“提不出两条边”进入到“提得出来，但不稳定”的阶段。

## 本轮不要偏离的修改方向

这一轮建议继续坚持“简洁、直接、可解释”的路线。

请优先围绕下面几个方向推进：

### Goal 1：稳定第一层 `body mask` 上的侧边拟合

当前侧边已经来自第一层 `body mask`，请继续在这一条线上收敛，不要重新把核心逻辑切回第二层 rim ROI。

优先关注：

- `body mask` 轮廓点选择是否过于贴近端部弧边
- 主轴方向估计是否足够稳定
- 左右侧点分组是否过宽，混入了非母线区域
- 当前阈值是否过于容易卡在门槛附近

### Goal 2：降低 `oblique / weak_side` 跳变

重点不是“强行放宽阈值”，而是：

- 找到为什么某些帧线能通过、某些帧刚好不过
- 让门控不要长期卡在阈值边缘
- 让 `side_lines` 的有效长度、拟合残差和主轴一致性更稳定

### Goal 3：继续增强调试可解释性

当前右侧调试窗已经扩到整根圆柱范围，这很好，请继续保留。

如果继续修改调试图，请优先保证它能回答：

- 当前线是从哪段 `body mask` 轮廓拟合出来的
- 哪一侧先失败
- 失败是长度不够、残差过大，还是方向不稳

不要为了“好看”而删掉这些关键诊断信息。

## 当前不建议做的事情

请暂时不要做下面这些事：

- 不要重新引入 tube/rim depth
- 不要回到“只看 bbox 主方向”的旧逻辑
- 不要重新堆一大套复杂的 RGB 搜索带精修结构，除非当前简化链路已经明确证明不够
- 不要靠时序平滑掩盖 `oblique / weak_side` 的跳变
- 不要急着把 side line 深度接入最终 pose 消歧
- 不要急着实现完整的：
  - `disambiguate_with_side_lines(...)`
  - `correct_projected_circle_center(...)`

## 当前建议的下一步计划

建议按下面顺序继续，而不是跳着做：

1. 先在当前简化结构上，把 `body mask` 侧边拟合稳定下来
2. 重点定位：
   - 端部弧边是否污染侧边点
   - 左右侧点截取范围是否合适
   - 质量门控哪一项最容易卡边
3. 在右侧整根圆柱调试窗中继续验证：
   - 线是否真的贴着期望的母线区域
   - 失败原因是否和图像现象一致
4. 只有在 `oblique` 激活足够稳定后，再考虑：
   - side line 轻量参与 pose candidate 评分
5. 最后才进入：
   - side line 真正参与 pose 消歧
   - projected circle center correction

## 当前阶段状态必须明确

请记住，当前阶段仍然应表述为：

- **侧边检测稳定化 + 可视化诊断增强**

而不是：

- **side-line 已经深度参与最终 pose 消歧**

## 修改后至少要完成的检查

1. `python3 -m py_compile src/vision_detection/vision_detection/config.py src/vision_detection/vision_detection/pose_estimator.py src/vision_detection/vision_detection/detection_node.py`
2. 最终说明里明确写出：
   - 改了哪些文件
   - 备份放在哪个目录
   - 当前 `oblique / weak_side` 跳变是否已经明显缓解
   - 当前是否仍处于“侧边稳定化阶段”

## 最终输出时必须说明

请在最终说明中明确写出：

- 本轮是否仍然只是“侧边检测稳定化与可视化增强”
还是
- 已经进入“side-line 真正参与 pose 消歧”

不要模糊表述阶段状态。
