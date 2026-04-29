# 4-23 更新记录：control 收口到最简链路

日期：2026-04-23

## 背景

在用户已经删除 `control` 包中大部分 D1 相关实现后，代码库里仍残留两类历史内容：

1. D1 遗留构建与启动内容
2. 候选法遗留源码、参数和话题

这些残留会带来两个直接问题：

- 构建脚本仍尝试引用已经不存在的源文件
- 执行器源码和参数表继续暴露一大批已经被判定无效的候选法变量

本轮目标是把 `control` 清成“只留下当前实际运行需要的最简集合”。

## 本次修改

### 1. 删除 D1 遗留构建分支

修改文件：

- `src/control/CMakeLists.txt`
- `src/control/package.xml`

处理：

1. 删除 `BUILD_D1_LEGACY`
2. 删除 `control_system_interface / pose_mover / new_control` 相关构建逻辑
3. 删除对已缺失源文件的引用
4. 移除不再需要的依赖：
   - `rclcpp_action`
   - `rclcpp_lifecycle`
   - `hardware_interface`
   - `pluginlib`
   - `sensor_msgs`
   - `trajectory_msgs`
   - `control_msgs`

结果：

- `control` 包的构建脚本现在只保留当前仍存在且仍在使用的目标

### 2. 删除 D1 遗留文件

删除文件：

- `src/control/launch/run_new_control_refactor.launch.py`
- `src/control/control_hardware_plugins.xml`

原因：

- `run_new_control_refactor.launch.py` 仍直接依赖 `d1_550_config`
- `control_hardware_plugins.xml` 只服务于已删除的 D1 ros2_control 插件

### 3. 将 `vision_moveit_executor.cpp` 重写为最简单目标执行器

修改文件：

- `src/control/src/vision_moveit_executor.cpp`

新的执行器只保留：

1. 订阅 `/target_pose`
2. 订阅 `/target_pose_stamped`
3. 对单个目标直接执行 `plan + execute`
4. 发布 `/vision_exec_status`
5. 保留最基本的：
   - 目标节流
   - 目标变化阈值
   - busy 时 latest-wins 排队

明确删除的候选法相关内容包括：

- `PoseArray` 订阅
- `target_pose_array_topic`
- `prefer_pose_array`
- 本地候选邻域构造
- `candidate_xy_step_m`
- `candidate_z_step_m`
- `candidate_roll_step_deg`
- `candidate_max_roll_steps`
- `max_candidate_plan_tries`
- `candidate_batch_max_count`
- `batch_rank_penalty`
- `feasible_cost_*`
- `enable_feasible_pose_projection`
- `camera_x_guidance_*`
- `relaxed_*` 姿态放松回退

也就是说，执行器已经不再保留“候选法源码只是通过参数关闭”的状态，而是从源码层面收口成单目标执行器。

### 4. 精简执行器参数文件

修改文件：

- `src/control/config/vision_moveit_executor.params.yaml`

现在保留的参数只有：

- 输入话题
- 状态话题
- latest-wins 排队
- 目标节流/位姿变化阈值
- MoveIt 基本规划参数

## 当前 control 包的最简集合

现在 `control` 包核心只剩：

- `vision_moveit_executor.cpp`
- `tf_compat_broadcaster.cpp`
- `run_agx_moveit_refactor.launch.py`
- `vision_bottom_scan_bringup.launch.py`
- 最小参数文件

## 结果

本轮之后：

1. `control` 里不再残留 D1 构建分支
2. 不再残留 D1 专用 launch / plugin 文件
3. 执行器不再包含候选法的源码和参数
4. 整个执行链路回到：
   - 单目标输入
   - 单次规划
   - 单次执行

这为后续重新设计“真正基于任务和可达性的策略”提供了一个更干净的起点。
