# 给下一位 Agent 的 Prompt

请基于当前仓库 `/home/arnoyin/agx_arm_ws`，继续实现 `vision_detection` 中“侧边精修 + 后续位姿消歧”的下一步工作。

开始前请先阅读：

- `archive/4-28update/CIRCLE_EXTRACTION_AND_POSE_OPTIMIZATION_PLAN_2026-04-28.md`
- `archive/4-28update/IMPLEMENTATION_LOG_2026-04-28.md`
- `archive/4-28update/side_line_refinement_design_2026-04-28/SIDE_LINE_REFINEMENT_MODULE_DESIGN_2026-04-28.md`
- `src/vision_detection/vision_detection/pose_estimator.py`
- `src/vision_detection/vision_detection/detection_node.py`

## 先执行的硬要求

在做任何代码修改前，必须先备份当前代码。

请先在 `archive/4-28update/` 下创建新的备份目录，例如：

- `archive/4-28update/code_backup_side_refine_<date-or-step>/`

并至少备份：

- `src/vision_detection/vision_detection/pose_estimator.py`
- `src/vision_detection/vision_detection/detection_node.py`
- `src/vision_detection/vision_detection/config.py`

这是硬要求，不要跳过。

## 总体计划不要偏离

请继续遵循总计划：

1. 纯 RGB 主导
2. 不把 tube/rim depth 重新引入主估计链路
3. 输出仍然保持：
   - `center_3d`
   - `axis_3d`
4. 不要靠时序平滑掩盖单帧几何误差
5. 不要把视觉结果重新包装成伪完整四元数姿态

## 当前进度

当前已经完成：

- rim 前端重构为 `boundary band + candidate + refit`
- `single circle -> pose candidates` 第一版
- `RGB-only` 选解
- `frontal / weak_side` 模式标记
- body-mask 侧边检测的第一版接入
- ROI 中 `S0/S1` 的基本标注

当前还没完成：

- 稳定可靠的侧边精修
- side-line 真正参与 `oblique` 模式的可靠切换
- `disambiguate_with_side_lines(...)`
- `correct_projected_circle_center(...)`

## 当前已知问题

请把下面这些问题视作你这轮工作的直接输入：

- 边缘提取仍有抖动，位姿仍有角度抖动
- `frontal` 模式下轴线会在两个候选之间来回跳
- `weak_side` 模式在大倾角时，红色 mask 与边缘提取仍可能不重合
- 当前侧边模式几乎无法激活
- 当前侧边可视化不够解释性

## 你当前最优先要做的事

请不要先急着做最终 side-line 几何修正。

这一轮最优先目标是：

### Goal 1：把“侧边检测”本身做稳

你需要把当前粗糙的：

- `body mask 极值点 -> fitLine`

替换为：

- `body mask 主轴估计`
- `左右窄搜索带生成`
- `回到 RGB 中做边缘精修`
- `左右侧边鲁棒拟合`
- `双侧一致性验证`

重点是：

- `body mask` 只做搜索约束，不直接当最终侧边
- 真正的侧边应尽量贴近 RGB 中的真实轮廓

### Goal 2：让调试图解释“为什么没进侧边模式”

请增强调试可视化，使其至少能区分：

- 没找到足够支持点
- 支持点找到了，但拟合失败
- 拟合成功，但一致性没过阈值
- 找到了两条线，但不应切入 `oblique`

建议加入：

- 左右 band 显示
- 左右支持点显示
- 候选线显示
- reject reason 显示

### Goal 3：只有在侧边检测真的可靠时，才切到 `oblique`

当前代码里：

- `len(side_lines) >= 2`

就可能把状态切到 `oblique`

请改成更严格的质量门控。建议综合：

- `span_ratio`
- `residual_px`
- `body_alignment`
- 左右线平行性
- 左右分离度
- 支持点数量

## 本轮建议交付范围

建议优先完成：

1. 重写 `detect_body_side_lines(...)`
2. 新增 body 主轴估计模块
3. 新增左右搜索带模块
4. 新增 RGB 侧边精修模块
5. 增强调试图与失败原因输出
6. 让 `observability_state = oblique` 建立在可信侧边基础上

如果时间足够，再继续：

7. 让 side lines 轻量参与 `pose candidate` 的选解评分

但如果侧边检测本身还不稳，不要急着上最终圆心修正。

## 不要做的事情

- 不要重新引入 tube/rim depth
- 不要靠时序平滑掩盖问题
- 不要只通过放宽几个阈值来“假装侧边模式激活了”
- 不要继续依赖 axis-aligned bbox 的水平/竖直方向来代表真实 body 主方向

## 你修改后至少要完成的检查

1. `python3 -m py_compile src/vision_detection/vision_detection/config.py src/vision_detection/vision_detection/pose_estimator.py src/vision_detection/vision_detection/detection_node.py`
2. 明确说明：
   - 改了哪些文件
   - 备份放在哪个目录
   - 当前侧边模式是否已经能够稳定激活
   - 还差哪些工作才能进入真正的 side-line 消歧

## 最终输出时必须说明

请在最终说明中明确写出：

- 本轮是否只是“侧边检测与可视化增强”
还是
- 已经进入“side-line 真正参与 pose 消歧”

不要模糊表述阶段状态。
