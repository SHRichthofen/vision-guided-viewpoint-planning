# 4-17 更新日志（从“半迁移”清理到 MoveIt2 真机链路诊断）

日期：2026-04-17  
工作区：`qrc_hand/arm_ws`  
目标：先修复上一版“半迁移”遗留问题，使系统回到标准执行链；再定位“RViz 执行成功但真机不动”的根因。

---

## 0. 本次更新前的“半迁移”问题基线

在 4-16 改造后，代码和配置处于“结构迁移了一半”的状态，核心矛盾是：

1. **架构目标是标准链路，但配置仍残留旧/仿真逻辑**
   - `ros2_control` 硬件插件仍可能落在 mock 语义。

2. **硬件接口与控制配置不完全一致**
   - 关节数量、状态接口（position/velocity）、hand 保留策略存在不一致风险。

3. **执行链存在“看起来能跑、实际上不闭环”的风险**
   - MoveIt 和 controller action 可成功，但底层执行与反馈一致性未被验证。

4. **DDS 运行库存在混用风险**
   - `/usr/local` 与 ROS 系统库并存，导致早期出现激活阶段断言崩溃。

本次日志按“先清理半迁移问题，再做真机闭环诊断”的顺序记录。

---

## 1. 本次工作概述

今天围绕“从半迁移到可诊断标准链路”做了四类工作：

1. **半迁移问题清理（配置/接口层）**
   - 将 `ros2_control` 硬件插件切换到 `control/D1SystemInterface`。
   - 对 `D1SystemInterface` 补齐 arm+hand 映射与 velocity 状态导出，匹配控制器期望接口。

2. **链路稳定化（运行时层）**
   - 修复 `ros2_control_node` 激活阶段 DDS 断言崩溃（P0/P1）。
   - 稳定 `controller_manager`、`joint_state_broadcaster`、`arm_controller` 启动流程。

3. **观测能力增强（不阻断执行）**
   - 在 `D1SystemInterface` 增加执行观测日志与状态发布，验证命令是否被真实反馈跟随。

4. **验证性策略尝试**
   - 增加“运动中周期性重使能”策略，验证“是否因为使能状态导致不动”。
   - 结果表明问题不在使能触发频率，而在更底层命令落地/反馈真实性。

---

## 2. 关键修改文件

### 2.1 主要修改

- `src/control/src/d1_system_interface.cpp`
   - 从“仅 arm 简化实现”扩展到与当前 joint/控制器配置一致的映射
   - 补齐 `velocity` 状态相关实现
   - 修复激活阶段稳定性问题（避免启动即崩）
  - 增加执行观测日志（命令-反馈误差、反馈年龄）
  - 增加 DDS 反馈解析（尝试提取 error/code/msg）
  - 增加 `/d1_hw_exec_status` 状态发布
  - 增加“运动中重使能”策略（验证用）

- `src/control/include/control/d1_system_interface.hpp`
   - 同步新增接口状态字段与观测成员

- `src/control/CMakeLists.txt`
   - 收敛 DDS 链接策略，降低混用库导致的不确定行为

- `src/control/launch/run_new_control.launch.py`
   - 保持标准链路启动顺序，配合控制器/MoveIt 执行路径验证

### 2.2 关联配置

- `src/d1_550_config/config/d1_550_description.ros2_control.xacro`
   - 硬件插件改为 `control/D1SystemInterface`
   - 本轮过程中对 `enable_on_activate` 做过开关验证（用于定位执行层问题）

- `src/d1_550_config/config/ros2_controllers.yaml`
   - 与接口导出能力（position/velocity）对齐

---

## 3. 运行现象与证据

## 3.1 半迁移问题修复后，标准链路可稳定启动

- `ros2_control_node` 可完成硬件 `init/configure/activate`
- `controller_manager/list_controllers` 服务可用
- `joint_state_broadcaster` 与 `arm_controller` 均可激活

说明：从“spawner 长时间等待服务”的半迁移异常，恢复到标准可启动状态。

## 3.2 MoveIt2 / 控制器 action 链路是通的

- `arm_controller` 能收到并接受 goal
- action 最终返回 `Goal reached, success`
- MoveIt 显示 `Execution completed: SUCCEEDED`

说明：软件层规划与 action 执行流程正常。

## 3.3 机械臂反馈不跟随目标

新增观测日志显示：

- `cmd` 角度明显变化（执行期间可到 40~55 deg 量级）
- `fb` 长时间保持在近似固定值（约 `[0.3, 0.4, 0.4, -0.1, 1.0, 0.1]`）
- `max_err` 持续增大并维持高位（约 40~55 deg）

结论：**命令发送成功，但反馈对应的机械状态没有发生匹配变化。**

## 3.4 “重使能”也不能驱动反馈变化

执行中周期性打印：

- `Motion enable refresh sent (...)`

但反馈依旧不变，说明“仅补发使能”不足以让机械臂动作。

---

## 4. 诊断结论（截至本次）

当前最可信结论：

1. **半迁移层面问题（配置不一致、启动不稳）已基本清理。**
2. **问题不在 MoveIt 规划层，也不在 JTC action 触发层。**
3. **当前主问题位于硬件执行落地链路或反馈链路真实性**：
   - 可能是命令未真正到达当前实机执行端；
   - 或反馈话题来源非实时实机（冻结/非同源）；
   - 或存在 DDS 侧实例/域/通道不一致导致“看似连通，实则未控制到目标设备”。

4. **“软件成功”与“硬件到位”目前仍是解耦状态**
   - 需要额外的命令-反馈一致性判据，不能仅依赖 action 成功。

---

## 5. 本次产出

1. 已形成可复用的**执行观测框架**（不影响执行）：
   - `ExecObs` 日志
   - `/d1_hw_exec_status` 状态话题
   - DDS 反馈 error/code/msg 尝试解析

2. 已将“是否是上层控制器问题”基本排除。
3. 已完成从“半迁移不可判定状态”到“可启动、可执行、可观测、可定位”的阶段跃迁。

---

## 6. 下一步建议

1. 做 A/B 同源验证：
   - A：仅 `ros2_control` 链路运行；
   - B：仅旧原生控制程序运行；
   - 对比同一时段 `current_servo_angle` 是否真实变化。

2. 排查 DDS 实例一致性：
   - domain/network/interface/进程并发发布者是否一致。

3. 加入“探针命令 + 反馈位移确认”机制（仅记录不阻断）：
   - 在激活后发送极小测试命令；
   - 用反馈变化判断“命令是否到达真实执行端”。

4. 增补“执行验收判据”文档：
   - action 成功 + 反馈位移阈值 + 超时策略
   - 将“到位判定”从经验观察变为可复现标准。

---

## 7. 备注

- 退出时 `move_group` 的 class_loader 相关 segfault 属于关停阶段现象，非本次“机械臂不动”主因。
- 本日志聚焦“MoveIt 执行成功但真机无动作”的链路诊断，不包含视觉/标定功能改动。
- 本次记录已包含“半迁移问题修复”到“真机不动根因定位”的完整链路，便于后续 4-18 继续接力。
