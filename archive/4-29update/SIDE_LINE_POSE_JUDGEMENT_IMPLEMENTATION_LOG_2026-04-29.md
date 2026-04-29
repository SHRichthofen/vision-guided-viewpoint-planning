# 2026-04-29 Side-Line 几何消歧实现记录

## 1. 几何关系重申

两条可见侧边是圆柱母线。

在 3D 世界中：

- 两条侧边母线彼此平行
- 两条侧边母线均沿圆柱轴线方向
- 圆柱口所在平面的法向也是圆柱轴线方向
- 因此侧边母线垂直于圆柱口所在平面

所以 side-line 的角色不是判断管口圆/椭圆是否成立，也不是参与 rim candidate 的质量评分。

side-line 当前只用于：

- 在单圆产生的两个位姿候选之间做几何消歧

## 2. 本轮目标修正

本轮根据最新要求，取消上一版“side-line 混入候选评分”的设计。

当前链路改为：

1. rim mask 只负责管口圆/椭圆提取
2. `score_candidates_rgb_only(...)` 只做基础 RGB/body 几何评分
3. `disambiguate_with_side_lines(...)` 在候选生成之后单独运行
4. 当 side lines 通过质量门控且不是 near-circle/frontal 模式时，用侧边母线方向直接选择 P0/P1

阶段性质应表述为：

- **side-line 直接参与单圆双解几何消歧**

而不是：

- **side-line 参与管口圆形判定**
- **side-line 混入 rim/candidate 质量评分**
- **side-line 已完成圆心投影修正**

## 3. 备份位置

本次修改前新增备份：

- `archive/4-29update/code_backup_before_side_geometric_disambiguation_2026-04-29/src/vision_detection/vision_detection/pose_estimator.py`
- `archive/4-29update/code_backup_before_side_geometric_disambiguation_2026-04-29/src/vision_detection/vision_detection/detection_node.py`
- `archive/4-29update/code_backup_before_side_geometric_disambiguation_2026-04-29/src/vision_detection/vision_detection/config.py`

上一轮备份仍保留：

- `archive/4-29update/code_backup_side_pose_judgement_2026-04-29/`

## 4. 修改文件

### `src/vision_detection/vision_detection/config.py`

移除上一版候选评分混入权重：

- `POSE_SIDE_LINE_SCORE_WEIGHT`
- `POSE_SIDE_AXIS_WEIGHT`
- `POSE_SIDE_BODY_DIRECTION_WEIGHT`
- `POSE_SIDE_CENTERLINE_WEIGHT`

新增几何消歧权重：

- `SIDE_DISAMBIG_AXIS_WEIGHT`
- `SIDE_DISAMBIG_BODY_DIRECTION_WEIGHT`

### `src/vision_detection/vision_detection/pose_estimator.py`

关键变化：

- `score_candidates_rgb_only(...)` 恢复为只使用基础 RGB/body 几何
- 新增 `disambiguate_with_side_lines(...)`
- `process_detection(...)` 中的顺序改为：
  1. `solve_circle_pose_candidates(...)`
  2. `score_candidates_rgb_only(...)`
  3. `disambiguate_with_side_lines(...)`
  4. `_finalize_pose_result_from_candidate(...)`

`disambiguate_with_side_lines(...)` 的核心逻辑：

- 从两条 side lines 得到图像中的母线方向
- 对每个单圆候选取其投影轴向 `axis_img_dir`
- 将母线方向朝向 body center / side pair midpoint
- 选择轴向最符合母线方向、且指向管身方向的候选

它不修改 rim candidate 分数，也不把 side line 当成圆/椭圆质量项。

### `src/vision_detection/vision_detection/detection_node.py`

语义 payload 继续透传：

- `side_line_pair_metrics`
- `side_disambiguation`

## 5. 调试输出

`pose_candidates_3d` 中每个候选新增或保留：

- `rgb_only_score`
- `side_disambiguation_score`
- `side_axis_alignment_score`
- `side_body_direction_score`
- `side_disambiguation_applied`
- `side_line_quality`

ROI debug 文本中，当 side-line 完成几何消歧时显示：

- `SideDisamb S=...`
- `Ax=...`
- `Tow=...`

`[SELECTED]` ROS 日志仅针对当前选中目标增强，不输出所有识别到的圆柱。

新增选中目标日志字段：

- `obs`
- `P`
- `side.applied`
- `side.score`
- `side.margin`
- `side.q`
- `side.status`
- `selected_pose_reason`
- `side_line_reject_reason`

这部分只在 `publish_selected_target()` 中输出，仍受 `enable_debug` 控制。

## 6. 当前边界

当前已经进入：

- **side-line 直接参与单圆双解几何消歧**

当前仍未进入：

- 基于极线/极点的 projected circle center correction
- 用 side-line 修正圆口中心投影
- 完整的最终位姿几何优化

继续保持：

- 纯 RGB 主导
- 不重新引入 tube/rim depth 主链路
- 视觉语义输出仍是 `center_3d + axis_3d`
- 不发布伪完整四元数姿态
- 不靠时序平滑掩盖单帧几何问题

## 7. 检查结果

已执行：

```bash
python3 -m py_compile src/vision_detection/vision_detection/config.py src/vision_detection/vision_detection/pose_estimator.py src/vision_detection/vision_detection/detection_node.py
```

结果：

- 通过

## 8. 当前闭环分析补充

根据后续实机日志和图片，当前问题重心已经从视觉侧 P0/P1 消歧转向控制层目标生成与 MoveIt 可达性。

新增记录：

- `archive/4-29update/CURRENT_ANALYSIS_POSSIBLE_ISSUES_2026-04-29.md`
- `archive/4-29update/NEXT_AGENT_WORKSPACE_PROGRESS_PROMPT_2026-04-29.md`

这些记录明确排除已经确认的问题项，包括手眼标定方向、相机安装朝向、`tcp_link/gripper_link/link6` 偏置、旧 calib bug、side-line 参与 rim 判定等。

当前最值得继续验证的是：

- 对近竖直管口，控制层“沿轴线退开固定距离并直视中心”的目标生成策略是否过于轴向化
- `camera_standoff_m=0.10` 是否对侧装相机过近
- 近竖直 `view_axis` 下 roll 选择是否造成 IK/MoveIt 难规划
- 视觉窗口紫色 `target` 是否仍在用旧可视化逻辑误导判断
