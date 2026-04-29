# arm_ws - D1 Arm Control Workspace

此工作空间包含用于 Unitree D1 机械臂的控制程序，核心功能由 `control` 功能包提供。

## 功能简介

主要控制节点为 `pose_mover`，它结合了 **MoveIt 2** 运动规划和 **Unitree DDS** 通信，实现了通过手柄（UDP 协议）或 ROS 话题对机械臂的实时控制。

### `pose_move` (pose_mover.cpp) 代码介绍

`pose_mover` 节点 (`pose_mover_node`) 是系统的核心控制器，主要功能如下：

1.  **多模式控制**：
    - **位姿控制模式 (Pose Control)**：通过逆运动学 (IK) 控制机械臂末端的空间位置 (X, Y, Z) 和姿态 (Roll, Pitch, Yaw)。
    - **关节控制模式 (Joint Control)**：直接控制各个关节的角度。

2.  **通信接口**：
    - **Unitree DDS**：直接通过 DDS 协议 (`rt/arm_Command`, `rt/arm_Feedback`) 与机械臂底层通信，发送指令并读取状态。
    - **ROS 2**：
        - 订阅 `/target_pose` 话题接收目标位姿。
        - 发布 `/joint_states` 供 MoveIt 和 RViz 可视化。
    - **UDP 手柄**：监听端口 `6969` 接收自定义协议的手柄数据。

3.  **安全机制**：
    - 组合键激活机制，防止误触。
    - IK 解算校验，检测关节跳变。
    - 自动同步目标位置以防止漂移。

## 运行指令

### 1. 编译代码

在工作空间根目录下执行：

```bash
cd /home/roverlzh/arm_ws
colcon build
source install/setup.bash
```

### 2. 运行程序

使用编写好的 launch 文件一键启动控制节点、MoveIt 和状态发布器：

```bash
# 让机械臂到达默认姿态（需要下载编译d1官方包）
cd d1/build/
./arm_zero_control
# 开启planning模块
ros2 launch control run_mover.launch.py
# 测试视觉通信是否正常
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: -0.1, y: 0.0, z: 0.35}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}" --once

```

## 手柄操作说明 (Gamepad Control)

程序启动后监听 UDP 端口 **6969** 等待手柄数据。

**基础安全操作：**
- **LT + A**：**激活/禁止** 机械臂移动（必须先激活才能进行其他操作）。
- **B**：使能关节 (Enable Joints)。

**常用功能：**
- **X**：回到原点 (Home Position)。
- **LB**：打开夹爪 (Open Gripper)。
- **RB**：关闭夹爪 (Close Gripper)。
- **Y**：**切换控制模式** (位姿模式 <-> 关节模式)。

**摇杆控制映射：**

| 输入 | 位姿模式 (Pose Mode) | 关节模式 (Joint Mode) |
| :--- | :--- | :--- |
| **左摇杆** (上下) | X 轴移动 | J2 关节 (大臂) |
| **左摇杆** (左右) | Y 轴移动 | J1 关节 (底座) |
| **右摇杆** (上下) | Z 轴移动 | J3 关节 (小臂) |
| **右摇杆** (左右) | Yaw 旋转 | J6 关节 (腕部旋转) |
| **十字键** | Pitch/Roll 旋转 | - |
