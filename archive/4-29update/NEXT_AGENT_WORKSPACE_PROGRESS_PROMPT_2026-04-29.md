# Next Agent Prompt: 2026-04-29 Workspace Progress

你接手的是 `/home/arnoyin/agx_arm_ws`。当前任务是把侧边线视觉几何接入圆柱目标姿态判定，并继续排查视觉到机械臂控制闭环中的目标位姿/MoveIt 可达性问题。

请先阅读：

- `archive/4-29update/SIDE_LINE_POSE_JUDGEMENT_IMPLEMENTATION_LOG_2026-04-29.md`
- `archive/4-29update/CURRENT_ANALYSIS_POSSIBLE_ISSUES_2026-04-29.md`
- `archive/4-28update/side_line_refinement_design_2026-04-28/NEXT_AGENT_SIDE_LINE_REFINEMENT_PROMPT_UPDATE_2026-04-28.md`

## 已完成状态

1. side-line 的几何关系已确定：
   - 两条侧边是圆柱母线
   - 两条侧边在 3D 中平行
   - 两条侧边均垂直于圆柱口所在面
   - side-line 只参与单圆 P0/P1 几何消歧，不参与管口圆/椭圆判定

2. 视觉代码已按上述原则修改：
   - `src/vision_detection/vision_detection/pose_estimator.py`
   - `src/vision_detection/vision_detection/detection_node.py`
   - `src/vision_detection/vision_detection/config.py`

3. 当前关键实现：
   - `score_candidates_rgb_only(...)` 只做基础 RGB/body scoring
   - `disambiguate_with_side_lines(...)` 在候选生成后单独用侧边做 P0/P1 消歧
   - `side_disambiguation` 被透传到 selected semantics
   - `[SELECTED]` 日志只针对当前选中目标输出，不打印所有检测圆柱

4. 已通过：

```bash
python3 -m py_compile src/vision_detection/vision_detection/config.py src/vision_detection/vision_detection/pose_estimator.py src/vision_detection/vision_detection/detection_node.py
```

5. 当前控制链路：
   - 视觉层发布 `/vision/selected_cylinder_semantics`
   - `vision_to_arm_transform.py` 把 `top_center_cam` 与 `axis_cam` 转为 base 下的 `top_center_base` 与 `axis_base`
   - `arm_pose_controller.py` 使用 `center + axis` 生成相机目标站位，再转成 tool pose 发布 `/target_pose_stamped`
   - `vision_moveit_executor.cpp` 订阅 `/target_pose_stamped` 并调用 MoveIt

## 不要重新怀疑的点

除非用户提供新的反证，不要把下面内容重新作为主要原因：

- 手眼标定整体方向错误
- `camera_link` 安装朝向错误
- `tcp_link`、`gripper_link`、`link6` 之间存在偏置
- 旧 calib bug
- side-line 应该参与 rim/circle 判定
- 当前主要问题是 P0/P1 在 selected 目标上频繁翻转
- `/cylinder_pose_base` 占位四元数导致控制层错误

用户已经确认 TF 可视化中 `camera_link` 朝向符合实物安装，`tcp_link`、`gripper_link`、`link6` 完全重合；calib 旧 bug 已修复。

## 当前实测现象

ID=366 的一次日志：

```text
vision center(cam) = [0.0266, -0.0295, 0.3290]
vision axis(cam)   = [-0.0140, 0.8584, 0.5128]
side               = applied True, score 1.00, margin 1.00, q 0.97

center(base)       = [0.3902, -0.0053, 0.1668]
axis(base)         = [-0.1280, 0.0306, -0.9913]
camera(base)       = [0.4030, -0.0084, 0.2659]
tool(base)         = [0.4003, 0.0345, 0.3235]
```

实物照片中，目标是前方木架上的竖直管口。当前控制层把它解释为近似竖直轴，要求相机在管口上方约 10 cm 做轴向俯视。这个目标多次 MoveIt 规划失败。

## 当前最值得继续做的事

优先从控制目标生成和可达性排查，不要先改视觉消歧。

建议顺序：

1. 记录执行前目标：
   - `/target_pose_debug`
   - `/target_pose_stamped`
   - `center_base`
   - `view_axis_base`
   - 规划失败原因和 MoveIt 返回信息

2. 对 ID=366 这种竖直管口，先尝试调参验证：
   - `src/vision_arm_control/config/arm_pose_controller_bottom_scan.params.yaml`
   - 将 `camera_standoff_m` 从 `0.10` 提高到 `0.16-0.22`
   - 保持视觉代码不变，比较 MoveIt 成功率

3. 若 standoff 后仍失败，实现更稳的控制层候选策略：
   - 对同一个 `center + axis` 生成多个 roll 候选
   - 对接近竖直轴的目标生成轻微斜向观察候选
   - 用 IK/MoveIt 可行性选择最终 `/target_pose_stamped`
   - 保持语义输入仍是 `center + axis`，不要让视觉层发布伪完整姿态

4. 若 MoveIt 成功但相机画面仍明显偏离管口轴向：
   - 执行后查 TF：`base_link -> camera_color_optical_frame`
   - 计算实际相机 optical +Z 与 `center_base - camera_position_base` 的夹角
   - 用这个角度判断是控制执行/姿态生成问题，还是视觉中心/轴向本身偏差

5. 视觉窗口中的紫色 `target` 暂时不要当成真实控制目标。
   - 它来自 `detection_node.py::draw_target_pose_info()`
   - 使用视觉节点本地 `scan_distance=0.05`
   - 与控制层 `camera_standoff_m=0.10` 和 tool pose 不是同一套逻辑
   - 后续可以把它改为显示 `/target_pose_debug` 投影，或明确标注为视觉本地预测

## 修改纪律

- 新备份和修改记录继续放在 `archive/4-29update/`
- 修改代码前先备份相关文件
- 不要删除已有 archive 记录
- 不要把 side-line 重新混入 rim/candidate 评分
- 不要用时序平滑掩盖单帧几何问题；如果做多帧，只用于选中目标后的一致性确认，并明确记录
- 修改后至少运行相应 Python 编译检查；C++ 修改则运行可用的 colcon/build 检查
