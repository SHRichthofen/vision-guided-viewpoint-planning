可以，一键启动已经包括新模块。

**一键 Bringup**
```bash
cd /home/arnoyin/agx_arm_ws
source install/setup.bash

ros2 launch control vision_bottom_scan_bringup.launch.py \
  arm_type:=piper \
  follow:=true \
  enable_view_goal_solver:=true \
  enable_legacy_pose_controller:=false
```

这会启动：

- `agx_arm_ctrl`
- MoveIt / RViz
- `vision_moveit_executor`
- 新模块 `view_goal_solver`
- `vision_detection`
- `vision_to_arm_transform`

旧的 `arm_pose_controller` 默认关闭，不会再走旧 `/target_pose_stamped` 主链路。

**分开看 Log 的推荐方式**
终端 1：只启动机械臂 + MoveIt，不启动新 solver/executor：

```bash
source install/setup.bash

ros2 launch control run_agx_moveit_refactor.launch.py \
  arm_type:=piper \
  follow:=true \
  enable_executor:=false \
  enable_view_goal_solver:=false
```

终端 2：单独启动 executor：

```bash
source install/setup.bash

ros2 run control vision_moveit_executor \
  --ros-args \
  --params-file /home/arnoyin/agx_arm_ws/src/control/config/vision_moveit_executor.params.yaml \
  -r joint_states:=/feedback/joint_states
```

终端 3：单独启动新 solver：

```bash
source install/setup.bash

ros2 run control view_goal_solver \
  --ros-args \
  --params-file /home/arnoyin/agx_arm_ws/src/control/config/view_goal_solver.params.yaml \
  -r joint_states:=/feedback/joint_states
```

终端 4：启动视觉检测 + 坐标转换，不启动旧 controller：

```bash
source install/setup.bash

ros2 launch vision_arm_control vision_arm_integration_refactor.launch.py \
  enable_legacy_pose_controller:=false
```

核心注意点：`view_goal_solver` 和 `vision_moveit_executor` 都用了自己的 `MoveGroupInterface`，分开启动时也必须 remap：

```bash
-r joint_states:=/feedback/joint_states
```

否则它们会默认监听 `joint_states`，又会出现 “latest received state has time 0.000000”。