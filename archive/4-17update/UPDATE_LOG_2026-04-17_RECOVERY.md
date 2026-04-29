# 4-17 补充更新日志（归零后失效恢复）

日期：2026-04-17  
工作区：`qrc_hand/arm_ws`  
结论：**机械臂已恢复，可稳定跟随 MoveIt RViz 执行**。

---

## 1. 问题现象

在一次成功执行后：
1. 通过 `Ctrl+C` 结束了控制节点与 RViz；
2. 运行 SDK 归零脚本重置机械臂；
3. 重新启动 MoveIt2 标准链路后出现“软件成功、真机不动”。

表现为：
- MoveIt 侧显示 `SUCCEEDED`；
- `D1SystemInterface` 持续打印 `Motion enable refresh sent (max_err=xx deg)`；
- 跟踪误差不收敛。

补充症状：
- 机械臂可被手轻易推动，停止后可停住，疑似处于低刚度/半使能状态。

---

## 2. 原因判断

本次定位结论：
1. 上层规划与 action 触发链路正常；
2. 问题在硬件执行状态恢复不完整；
3. 归零脚本后，底层驱动模式可能未回到当前 `ros2_control` 期望状态。

---

## 3. 本次修复内容

### 3.1 增加使能恢复参数与激活脉冲机制

修改文件：
- `src/control/include/control/d1_system_interface.hpp`
- `src/control/src/d1_system_interface.cpp`

新增参数：
- `enable_mode`（默认 `80000`）
- `enable_pulse_on_activate`（默认 `true`）
- `enable_pulse_delay_ms`（默认 `120`）

行为变更：
- `on_activate()` 阶段支持：`disable -> delay -> enable` 脉冲恢复；
- 使能命令不再硬编码 `80000`，改为可配置 `enable_mode`。

### 3.2 诊断日志开关保留

此前已新增并沿用：
- `enable_exec_obs_log`
- `enable_diag_p1_log`

当前配置默认关闭，避免干扰其他节点观察。

### 3.3 硬件参数写入 xacro

修改文件：
- `src/d1_550_config/config/d1_550_description.ros2_control.xacro`

已加入：
- `<param name="enable_mode">80000</param>`
- `<param name="enable_pulse_on_activate">true</param>`
- `<param name="enable_pulse_delay_ms">120</param>`

---

## 4. 验证结果

1. 重新编译：
   - `control`
   - `d1_550_config`
2. 重新启动后，机械臂可正常跟随 MoveIt RViz 轨迹执行；
3. 本轮结论：**归零后失效问题已恢复**。

---

## 5. 经验与建议

1. 在运行 SDK 归零脚本后，建议始终完整重启控制链路；
2. 保留激活脉冲恢复机制，作为底层状态机回切保险；
3. 如后续复发，优先 A/B 测试 `enable_mode`，再评估是否增加更严格握手验证。
