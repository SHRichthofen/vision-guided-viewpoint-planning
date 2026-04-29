# 4-16 更新日志（MoveIt2 严格对齐改造）

日期：2026-04-16  
工作区：qrc_hand/arm_ws  
目标：将现有“自定义 topic + 节点内规划执行”控制链，升级为“MoveIt2 + ros2_control + SystemInterface 硬件插件”标准链路，并移除手柄相关路径（暂不支持 hand 控制）。

---

## 1. 本次改造目标与完成情况

### 1.1 总体目标

1. MoveIt2 除规划外，可通过 RViz 的标准执行链路直接驱动真机。
2. 控制接口与 MoveIt2 严格对齐：`FollowJointTrajectory` → `joint_trajectory_controller` → `SystemInterface`。
3. 去除手柄控制路径，简化为标定优先架构。
4. 暂时不纳入 hand（夹爪）控制，先稳定 arm 主链路。

### 1.2 按阶段完成度（对应计划 Phase 1-4）

- Phase 1（拆分旧控制逻辑）：**已完成**  
  新建节点并移除手柄依赖，旧路径与新标准路径分离。

- Phase 2（硬件适配层）：**已完成（可编译可加载版本）**  
  已实现 `SystemInterface` 插件，完成 DDS 读写与关节接口导出。

- Phase 3（控制器标准化）：**已完成（arm-only）**  
  `ros2_controllers.yaml` 与 `moveit_controllers.yaml` 已收敛为 arm-only。

- Phase 4（MoveIt2 执行链对齐）：**已完成（配置级）**  
  启动链已切为 `ros2_control_node + spawner + move_group` 标准模式。

---

## 2. 变更文件清单

### 2.1 新增文件

1. `control` 包新节点（过渡桥）
   - [src/control/src/new_control.cpp](src/control/src/new_control.cpp)

2. `ros2_control` 硬件插件
   - [src/control/include/control/d1_system_interface.hpp](src/control/include/control/d1_system_interface.hpp)
   - [src/control/src/d1_system_interface.cpp](src/control/src/d1_system_interface.cpp)
   - [src/control/control_hardware_plugins.xml](src/control/control_hardware_plugins.xml)

3. 新启动文件（标准控制链）
   - [src/control/launch/run_new_control.launch.py](src/control/launch/run_new_control.launch.py)

### 2.2 修改文件

1. 控制包构建与依赖
   - [src/control/CMakeLists.txt](src/control/CMakeLists.txt)
   - [src/control/package.xml](src/control/package.xml)

2. ros2_control 硬件 xacro（从 mock 切到真实插件）
   - [src/d1_550_config/config/d1_550_description.ros2_control.xacro](src/d1_550_config/config/d1_550_description.ros2_control.xacro)

3. 控制器配置（去除 hand）
   - [src/d1_550_config/config/ros2_controllers.yaml](src/d1_550_config/config/ros2_controllers.yaml)
   - [src/d1_550_config/config/moveit_controllers.yaml](src/d1_550_config/config/moveit_controllers.yaml)

---

## 3. 代码设计思路（核心）

## 3.1 旧版本问题

旧版核心控制集中在 [src/control/src/pose_mover.cpp](src/control/src/pose_mover.cpp)：

1. 自定义入口：`/target_pose` topic + 节点内部 `MoveGroupInterface` 规划。
2. 执行侧绕过 `ros2_control` 与 `joint_trajectory_controller`。
3. 手柄逻辑、规划逻辑、DDS 执行逻辑高度耦合。
4. RViz MoveIt 面板“标准执行”与真机控制链不一致。

## 3.2 新版本总体架构

当前改造后的标准链路：

RViz / MoveIt → `move_group` → `arm_controller`(`joint_trajectory_controller`) → `controller_manager` → `control/D1SystemInterface` → Unitree DDS → 真机

### 关键思想

1. **控制器负责轨迹 action 与插值**，硬件插件只做 `read()` / `write()`。
2. **硬件插件导出标准 `position` 命令与状态接口**，与 MoveIt2 原生兼容。
3. **DDS 适配下沉到硬件层**，应用节点不再承担执行中枢。
4. **先 arm-only 收敛稳定性**，后续再扩展 hand。

---

## 4. 关键实现说明

## 4.1 `D1SystemInterface` 关键职责

位于 [src/control/src/d1_system_interface.cpp](src/control/src/d1_system_interface.cpp)：

1. `on_init()`
   - 校验关节数量（当前按 6 轴 arm）
   - 建立关节名到索引映射

2. `on_configure()`
   - 初始化 Unitree DDS 通道（命令/反馈/伺服角）

3. `on_activate()`
   - 令命令缓存与状态对齐，避免激活瞬间跳变
   - 可配置是否发送使能命令

4. `read()`
   - 从 `current_servo_angle` 更新 `hw_states_`

5. `write()`
   - 将 `hw_commands_` 转换为 DDS `angle0~angle5`
   - `angle6` 维持当前值（hand 暂未纳入控制器）

## 4.2 配置收敛

1. 硬件插件从 `mock_components/GenericSystem` 改为 `control/D1SystemInterface`。
2. `ros2_controllers.yaml` 仅保留：
   - `joint_state_broadcaster`
   - `arm_controller` (`Joint1~Joint6`)
3. `moveit_controllers.yaml` 仅保留 `arm_controller` 的 `FollowJointTrajectory`。

---

## 5. 详细使用教程（arm-only）

以下流程用于“RViz 拖动/规划执行到真机”基础验证。

## 5.1 编译

在 `arm_ws` 根目录执行：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select control d1_550_config --symlink-install
source install/setup.bash
```

## 5.2 启动标准控制链

```bash
ros2 launch control run_new_control.launch.py
```

该启动文件位置：
- [src/control/launch/run_new_control.launch.py](src/control/launch/run_new_control.launch.py)

会启动：
1. `robot_state_publisher`
2. `ros2_control_node`
3. `joint_state_broadcaster`（spawner）
4. `arm_controller`（spawner）
5. `move_group`

## 5.3 检查控制器状态

```bash
ros2 control list_controllers
```

期望至少包含：
1. `joint_state_broadcaster`（active）
2. `arm_controller`（active）

## 5.4 检查 action 接口

```bash
ros2 action list | grep follow_joint_trajectory
```

期望出现：
- `/arm_controller/follow_joint_trajectory`

## 5.5 在 RViz 中执行

1. 打开 MoveIt RViz（使用 d1_550_config 默认配置）。
2. 在 MotionPlanning 面板拖动目标位姿。
3. 点击 `Plan` 确认轨迹可解。
4. 点击 `Execute`，轨迹应通过 `arm_controller` 下发到真机。

## 5.6 常见问题排查

1. 找不到 `arm_controller/follow_joint_trajectory`
   - 检查 `arm_controller` 是否 active
   - 检查 [src/d1_550_config/config/moveit_controllers.yaml](src/d1_550_config/config/moveit_controllers.yaml)

2. MoveIt 可规划但不动
   - 检查 `ros2_control_node` 是否成功加载 `control/D1SystemInterface`
   - 检查 DDS 通道是否正常

3. 关节名不匹配
   - 统一核对 URDF/SRDF/controller/moveit 配置中的关节命名顺序

---

## 6. 与前一版本对比（优缺点）

## 6.1 前一版本（pose_mover 主导）的优点

1. 一体化控制，开发初期上手快。
2. 自定义逻辑灵活，手柄+IK 调试方便。
3. 可直接通过 topic 推位姿测试。

## 6.2 前一版本的缺点

1. 非标准执行链，与 MoveIt2 原生工作流耦合弱。
2. 规划、输入、执行、状态发布耦合过重，维护复杂。
3. RViz 标准 `Plan/Execute` 与真实执行通路不完全对齐。
4. 后续扩展（Servo、控制器容差、统一监控）难度高。

## 6.3 当前版本（SystemInterface 架构）优点

1. 与 MoveIt2/ros2_control 标准接口严格对齐。
2. 责任分层清晰：规划层/控制器层/硬件层解耦。
3. 更适配 RViz 交互执行与标定工作流。
4. 后续可平滑接入 MoveIt Servo、监控、容差调优。

## 6.4 当前版本的不足与代价

1. 初始接入复杂度高于单节点方案。
2. 对配置一致性（关节名、控制器、action）要求更严格。
3. 目前是 arm-only，hand 功能尚未纳入标准控制链。

---

## 7. 本次结果总结

1. 已完成从“自定义执行”向“MoveIt2 标准执行”的关键架构迁移。
2. 已形成可编译、可启动、可验证的 `SystemInterface` 控制链骨架。
3. 当前版本优先保障标定效率所需的 RViz 标准执行能力。
4. 后续建议在该架构上继续做：
   - 实机容差与速度参数调优
   - MoveIt Servo 实时微调接入
   - hand 控制链按同样标准补齐

---

## 8. 建议的下一轮任务

1. 完成一次全流程实机验收（包含 20+ 目标点连续执行）。
2. 记录执行误差与超时日志，收敛控制器参数。
3. 增补“标定专用操作流程文档”（一键启动 + 采样规范）。
4. 在稳定后再恢复/重构 hand 控制器与 gripper action。
