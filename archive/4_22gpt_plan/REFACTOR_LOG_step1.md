# Step 1 重构日志：视觉语义补全 + 候选观察位姿生成

## 本次目标
本轮不直接推翻现有话题接口，而是先做兼容式重构：
1. 保留 `/vision/cylinder_pose`、`/cylinder_pose_base`、`/target_pose` 的原有兼容链路。
2. 在视觉层补出“可用于搜索决策的语义信息”。
3. 在控制层把“单个轴向退开位姿”改成“一组围绕底部 ROI 的候选观察位姿”。
4. 同步补齐调试与日志输出，方便项目报告复盘。

---

## 修改文件
- `config.py`
- `detection_node.py`
- `arm_pose_controller.py`

---

## 1. detection_node.py 改了什么

### 1.1 新增语义发布话题
新增 `std_msgs/String` 话题：
- `/vision/selected_cylinder_semantics`

发布内容为 JSON，核心字段包括：
- `track_id`
- `top_center_cam`
- `axis_cam`
- `axis_ratio`
- `quality_score`
- `near_circle_degenerate`
- `z_ellipse / z_prior / z_fused / z_fusion_weight`
- `depth_prior_confidence`
- `quality_components`
- `depth_prior_metrics`

### 为什么这样改
原系统只向下游发布 `PoseStamped(center + normal)`。这对“末端朝哪里去”勉强够用，但对“候选搜索该扩多大、该不该改成斜视角、当前观测是否退化”完全不够。
这次把质量分数、近圆退化、深度融合信息显式发布出来，下游控制层才能真正做自适应搜索。

### 1.2 默认关闭 `publish_once_per_selection`
把视觉侧 `publish_once_per_selection` 的默认值改为 `False`。

### 为什么这样改
原来的“选一次只发一次”更像单次抓取，不适合扫描任务。扫描任务本质上更接近闭环观察，后续视觉更新应该继续驱动控制层修正目标。

### 1.3 低质量姿态冻结策略从“只冻结法向”升级为“可选冻结完整 pose”
新增参数：
- `freeze_full_pose_on_low_quality`

当质量低于 `quality_min_full_pose` 且有历史高质量观测时，默认冻结完整姿态（中心、法向、椭圆）而不是只冻结法向，同时保留本帧的质量与深度诊断字段。

### 为什么这样改
旧逻辑会产生“旧法向 + 新位置”的混搭状态。对后续按轴向生成观察位姿的控制层来说，这会把目标放到一个几何上不自洽的位置。由于任务对象是静态圆柱，低质量时冻结完整 pose 更稳。

### 1.4 rim mask 选择由“最大面积”改成“面积 + 居中度联合评分”
新增 `_select_best_rim_mask()`：
- 面积大的 mask 依然优先
- 但如果 mask 远离 crop 中心，会被扣分

### 为什么这样改
当前 crop 是围绕 body 检测框得到的，所以真正的 rim 更应该靠近 crop 中心。只按面积最大选，很容易在复杂背景或错误分割时选到无关区域。

---

## 2. arm_pose_controller.py 改了什么

### 2.1 控制目标从“单个轴向退开位姿”改成“候选观察位姿集合”
新增默认模式：
- `target_mode = bottom_ring_candidates`

核心思路：
1. 把视觉给出的 rim 中心视作 `top_center`
2. 用 `axis_into_tube_sign * axis_raw` 定义“朝向底部的轴向”
3. 根据 `tube_length_m` 估计底部 ROI
4. 在 top_center 外侧生成一个观察环（ring）
5. 每个候选都 look-at 到 `bottom_roi`
6. 通过手眼标定把相机候选位姿换算成末端候选位姿
7. 选分数最高的候选发布到 `/target_pose`

### 为什么这样改
旧版控制器本质上是在“沿轴线直进/直退”。这更像“靠法向对准”，不是真正的“找能看到底部的相机位姿”。
新版本改成围绕 bottom ROI 生成观察候选，终于把任务目标“看到底部”写进了位姿生成逻辑。

### 2.2 新增视觉语义订阅
新增订阅：
- `/vision/selected_cylinder_semantics`

控制层会读取：
- `quality_score`
- `near_circle_degenerate`

并据此动态扩展候选搜索：
- 低质量时扩大半径与 azimuth 数量
- near-circle 时进一步增加 azimuth 覆盖

### 为什么这样改
当椭圆接近正圆时，轴向估计本来就更不稳定，这时应该主动扩大搜索而不是继续相信单点目标。

### 2.3 新增候选位姿发布与调试日志
新增话题：
- `/target_pose_candidates`：所有候选末端位姿（PoseArray）
- `/target_pose_debug`：JSON 调试报告

### 为什么这样改
原系统只看得到最后一个目标位姿，无法回答“为什么它会这么选”。
现在候选和评分都能留痕，更适合做项目报告中的方法说明和失败案例分析。

---

## 3. 新增/调整的重要参数

### 视觉层
- `publish_once_per_selection = False`
- `freeze_full_pose_on_low_quality = True`
- `selected_semantics_topic = /vision/selected_cylinder_semantics`

### 控制层
- `target_mode = bottom_ring_candidates`
- `tube_length_m`
- `bottom_roi_offset_m`
- `axis_into_tube_sign`
- `camera_entry_clearance_m`
- `camera_standoff_m`
- `candidate_radius_list_m`
- `candidate_axial_offset_list_m`
- `base_azimuth_count`
- `near_circle_extra_azimuth`
- `low_quality_radius_scale`
- `high_quality_radius_scale`

---

## 4. 本轮仍然保留的问题
1. 执行层 `vision_moveit_executor.cpp` 仍然只消费单个目标 pose，还没有直接消费候选集。
2. 视觉深度先验仍主要来自 body mask，不是 rim 邻域深度。
3. `axis_into_tube_sign` 仍需要现场确认一次，确定“视觉轴向正方向是否真的朝底部”。
4. 还没有利用当前机械臂实时位姿对候选进行运动代价排序。

---

## 5. 下一轮建议改造
下一轮优先改 `vision_moveit_executor.cpp`：
1. 订阅 `/target_pose_candidates` 或新的候选消息
2. 对候选先做 IK / 碰撞 / 简单可达性预筛
3. 再把前 2~3 个交给 MoveIt 规划
4. 失败后换候选，而不是只在单个坏目标附近抖动

---

## 6. 建议补充上传的文件
为了继续完成 Step 2，我最希望看到：
1. MoveIt 的 `move_group` / OMPL / controller 配置文件
2. 机械臂 URDF / SRDF
3. 当前使用的手眼标定 `calib.yaml`
4. 如果有的话，末端相机安装示意或 `camera_link -> tool` 的真实定义
5. 如果执行层有 launch 文件，也一起上传
