# 多圆柱目标选择与队列管理系统

## 系统概述

升级后的系统支持**多圆柱检测、UI选择、队列管理、状态监听**的完整工作流，解决了之前多个圆柱时机械臂反应错乱的问题。

## 架构图

```
┌──────────────────────────────────────────────────────────────────────────┐
│                          RealSense D435i Camera                          │
└───────────────────────────┬────────────────────────────────────────────┘
                            │ RGB Stream
                            ▼
┌──────────────────────────────────────────────────────────────────────────┐
│              vision_detection (YOLO Stage 1 + Stage 2)                   │
│  ┌────────────────────────────────────────────────────────────┐          │
│  │ • Detect all cylinders (Stage 1: body)                     │          │
│  │ • Crop + detect rims (Stage 2: rim/hole)                   │          │
│  │ • Estimate 3D pose + smooth + aggregate                    │          │
│  │ • Publish ALL cylinders                                    │          │
│  └────────────────────────────────────────────────────────────┘          │
└────────────────────┬─────────────────────────────────────────────────────┘
                     │ /vision/cylinders (PoseArray)
                     │ [所有检测到的圆柱+ID]
                     ▼
    ┌────────────────────────────────────────────┐
    │  target_selector (NEW - Brain Layer)       │
    │  ┌──────────────────────────────────────┐  │
    │  │ UI Window (OpenCV)                   │  │
    │  │ • Display all cylinders              │  │
    │  │ • Highlight selected target          │  │
    │  │ • Show execution queue               │  │
    │  │                                      │  │
    │  │ Controls:                            │  │
    │  │ • UP/DOWN: Select cylinder           │  │
    │  │ • ENTER: Confirm                     │  │
    │  │ • A: Toggle Auto Mode                │  │
    │  │ • Q: Quit                            │  │
    │  └──────────────────────────────────────┘  │
    │  ┌──────────────────────────────────────┐  │
    │  │ State Monitor                        │  │
    │  │ • Listen /joint_states               │  │
    │  │ • Detect arm moving/stopped          │  │
    │  │ • Timeout protection                 │  │
    │  │ • Auto trigger next in queue         │  │
    │  └──────────────────────────────────────┘  │
    │  ┌──────────────────────────────────────┐  │
    │  │ Execution Queue Management           │  │
    │  │ • mode: manual / auto                │  │
    │  │ • queue: [cyl_id_0, cyl_id_1, ...]   │  │
    │  │ • current: cyl_id_0                  │  │
    │  │ • status: ready/executing/done       │  │
    │  └──────────────────────────────────────┘  │
    └────────────┬─────────────────────────────┘
                 │ /target_id (Int32)
                 │ [选中的圆柱ID]
                 ▼
    ┌────────────────────────────────────────────┐
    │  vision_to_arm_transform (NO CHANGE)       │
    │  ┌──────────────────────────────────────┐  │
    │  │ Listen:                              │  │
    │  │  /vision/cylinder_pose               │  │
    │  │  /target_id                          │  │
    │  │                                      │  │
    │  │ Process:                             │  │
    │  │ • Only process when target_id set    │  │
    │  │ • Apply hand-eye calibration         │  │
    │  │ • Transform: camera → tool0          │  │
    │  │                                      │  │
    │  │ Publish:                             │  │
    │  │ /cylinder_pose_base (tool0 frame)    │  │
    │  └──────────────────────────────────────┘  │
    └────────────┬─────────────────────────────┘
                 │ /cylinder_pose_base (PoseStamped)
                 │ [工具坐标系中的圆柱位姿]
                 ▼
    ┌────────────────────────────────────────────┐
    │  arm_pose_controller (MODIFIED)            │
    │  ┌──────────────────────────────────────┐  │
    │  │ Subscribe to:                        │  │
    │  │  • /cylinder_pose_base               │  │
    │  │  • /target_id                        │  │
    │  │                                      │  │
    │  │ Generate target pose:                │  │
    │  │ • Only for selected cylinder         │  │
    │  │ • Offset: 5cm along normal vector    │  │
    │  │ • Camera pointing towards rim        │  │
    │  │                                      │  │
    │  │ Publish: /target_pose                │  │
    │  └──────────────────────────────────────┘  │
    └────────────┬─────────────────────────────┘
                 │ /target_pose (PoseStamped)
                 │ [机械臂目标位姿]
                 ▼
            ┌────────────┐
            │   MoveIt   │
            │ Planning & │
            │ Execution  │
            └────────────┘
```

## 关键改动

### 1. vision_detection 节点

**改动**：从发布单个PoseStamped改为发布PoseArray

```python
# 旧：单个发布
self.pose_pub.publish(pose_msg)  # /vision/cylinder_pose

# 新：数组发布
pose_array = PoseArray()
for each_cylinder:
    pose_array.poses.append(pose)
self.cylinders_pub.publish(pose_array)  # /vision/cylinders (PoseArray)
```

**新增功能**：
- 订阅 `/target_id (Int32)` 来获知用户选择
- 当收到选择时，同时发布该圆柱到 `/vision/cylinder_pose (PoseStamped)`
- 保持向后兼容性

### 2. arm_pose_controller 节点

**改动**：订阅 `/target_id` 而不是直接订阅位姿

```python
# 旧：直接处理任何接收到的位姿
self.cylinder_pose_sub = self.create_subscription(...)
def on_cylinder_pose_callback(...):
    # 立即生成并发布目标

# 新：只处理被选中的圆柱
self.target_id_sub = self.create_subscription(Int32, '/target_id', ...)
self.cylinder_pose_sub = self.create_subscription(...)
def on_cylinder_pose_callback(...):
    if self.selected_target_id is not None:
        self.generate_target()
```

### 3. target_selector 节点 (NEW)

**主要功能**：

#### 3.1 UI界面（OpenCV）
```
┌─────────────────────────────────────┐
│ Cylinder Target Selector            │
│ Detected Cylinders: 3               │
│                                     │
│ >>> [SELECTED] <<< Cylinder 0       │
│ Pos=(0.450, 0.120, 0.320)           │
│                                     │
│ [ ] Cylinder 1                      │
│ Pos=(0.280, -0.150, 0.305)          │
│                                     │
│ [ ] Cylinder 2                      │
│ Pos=(0.620, 0.080, 0.315)           │
│                                     │
│ ─────────────────────────────────── │
│ UP/DOWN or W/S - Select cylinder    │
│ ENTER - Confirm   A - Auto  Q - Quit│
│ Status: MANUAL MODE                 │
└─────────────────────────────────────┘
```

#### 3.2 状态监听
- 订阅 `/joint_states` 监听机械臂运动状态
- 检测机械臂是否在运动（velocity norm > threshold）
- 超时保护：10秒无进展后告警

#### 3.3 队列管理
- **手动模式**（默认）：用户手动选择圆柱，逐个操作
- **自动模式**（A键切换）：自动生成队列，机械臂到位后自动进行下一个

**工作流**：
```
手动模式：
  1. 用户按UP/DOWN选择圆柱
  2. 发布 /target_id → arm_controller
  3. arm_controller 生成目标位姿
  4. MoveIt规划并执行
  5. 用户观察视觉反馈，判断是否完成
  6. 重复步骤1-5选择下一个

自动模式：
  1. 初始化：queue = [cyl_0_id, cyl_1_id, cyl_2_id, ...]
  2. 发布 queue[0] → arm_controller
  3. 监听 /joint_states
  4. 当机械臂速度 < threshold 持续2秒 → 认为到位
  5. queue.pop(0)，发布 queue[0]
  6. 重复3-6直到queue为空
  7. 如果超时(10s无运动) → PAUSE，等待人工确认
```

## 数据流示例

### 场景：3个圆柱，手动逐个操作

```
t=0s: 
  RealSense capture
    ↓
  YOLO detection: found 3 cylinders
    ↓
  Publish /vision/cylinders (3 poses)
    ↓
  target_selector UI updates
    ┌─────────────────────────────────┐
    │ Detected Cylinders: 3           │
    │ >>> [SELECTED] Cylinder 0 (ID=0)│
    │ [ ] Cylinder 1 (ID=1)           │
    │ [ ] Cylinder 2 (ID=2)           │
    └─────────────────────────────────┘

t=1s: User presses DOWN arrow
  target_selector: selected_idx = 1
    ↓
  Publish /target_id = 1
    ↓
  vision_detection: on_target_selected(1)
    • Publish /vision/cylinder_pose (ID=1 pose only)
    ↓
  arm_pose_controller: on_target_id_callback(1)
    • selected_target_id = 1
    ↓
  arm_pose_controller: on_cylinder_pose_callback(cylinder_1_pose)
    • Generate target: 5cm along normal
    • Publish /target_pose
    ↓
  MoveIt: plan & execute
    ↓
  Arm moves to cylinder 1

t=2s: User presses DOWN arrow again
  target_selector: selected_idx = 2
    ↓
  Publish /target_id = 2
    ↓
  [Same flow for cylinder 2]
```

## 启动系统

### 新的集成Launch文件

```bash
ros2 launch vision_arm_control vision_arm_integration.launch.py
```

启动的节点：
1. `vision_detection` - YOLO + RealSense
2. `target_selector` - UI + 状态监听 + 队列管理
3. `vision_to_arm_transform` - 坐标变换
4. `arm_pose_controller` - 位姿生成

### 手动分离启动（调试用）

```bash
# Terminal 1: Vision detection
ros2 run vision_detection cylinder_detection

# Terminal 2: UI + 队列管理
ros2 run vision_arm_control target_selector

# Terminal 3: Transform
ros2 run vision_arm_control vision_to_arm_transform

# Terminal 4: Controller
ros2 run vision_arm_control arm_pose_controller
```

## 关键参数

### target_selector
```yaml
enable_ui: true              # 启用UI界面
ui_scale: 1.0               # UI缩放比例
auto_mode: false            # 自动模式（用户可通过A键切换）
arm_reach_threshold: 0.01   # 10mm - 判定到位的阈值
arm_timeout_seconds: 10.0   # 超时保护时间
```

### arm_pose_controller
```yaml
scan_distance: 0.05         # 5cm - 距圆柱口的距离
enable_debug: true
target_frame: 'tool0'
```

## 优势对比

| 方面 | 旧系统 | 新系统 |
|-----|------|------|
| **多圆柱处理** | ❌ 不支持 | ✅ 全部检测 |
| **用户控制** | ❌ 无 | ✅ UI选择 + 手动/自动模式 |
| **状态监听** | ❌ 无 | ✅ 监听joint_states |
| **安全保护** | ❌ 无 | ✅ 超时保护 + 人工干预 |
| **队列管理** | ❌ 无 | ✅ 自动/手动队列 |
| **反应责错** | ❌ 多圆柱冲突 | ✅ 单目标确定 |
| **易用性** | ❌ 需修改代码 | ✅ UI界面 |

## 工作流决策树

```
用户选择模式
    ├─ 手动模式（默认）
    │  ├─ 看UI界面选择圆柱
    │  ├─ 按UP/DOWN切换
    │  ├─ 发送/target_id
    │  ├─ arm_controller生成目标
    │  ├─ MoveIt执行
    │  └─ 重复
    │
    └─ 自动模式（按A键切换）
       ├─ 初始化：queue = all_cylinders
       ├─ 发送queue[0]→arm_controller
       ├─ 监听/joint_states
       ├─ 判定到位条件：
       │  ├─ 速度<阈值持续2秒 → 到位
       │  └─ 超时10秒 → 暂停告警
       ├─ queue.pop(0)
       └─ 重复
```

## 故障恢复

### 情况1：机械臂卡住
```
监测：超时10秒无运动
动作：
  1. target_selector 停止发送新target_id
  2. 用户在UI中看到 "TIMEOUT - PAUSED"
  3. 用户按 Q 退出或手动干预
  4. 重新启动节点或按ENTER继续
```

### 情况2：视觉漂移
```
现象：检测到的位置不稳定
原因：RealSense或光照变化
解决：
  1. vision_detection已内置平滑器（PoseSmoother）
  2. 30-sample聚合（PoseAggregator）
  3. 用户可在UI中重新选择以刷新
```

### 情况3：多圆柱误操作
```
原因：用户选错了圆柱
现象：机械臂去错地方
防护：
  1. arm_controller只处理当前selected_target_id
  2. 用户可立即在UI中切换目标
  3. MoveIt可停止当前规划（SIGINT）
```

## 未来扩展

### Phase 2（已设计，待实现）
- [ ] 自动模式下的队列持久化
- [ ] Web界面而非OpenCV UI
- [ ] 实时3D可视化（RViz集成）
- [ ] 圆柱识别置信度阈值过滤

### Phase 3（可选）
- [ ] QR码扫描定位
- [ ] 多角度扫描策略
- [ ] 机械臂碰撞检测集成
- [ ] ROS2 Actions for 长时间操作

## 测试清单

- [ ] 单圆柱正常操作
- [ ] 多圆柱UI显示正确
- [ ] 手动模式选择正常
- [ ] 自动模式队列正常
- [ ] 超时保护工作
- [ ] 坐标变换正确
- [ ] arm_controller只处理选中目标
- [ ] 编译无错误

---

**版本**: 2.0 - Multi-Cylinder Selection System  
**日期**: 2026-03-24  
**状态**: Ready for Testing
