# 手眼标定自动采集指南

## 功能说明

修改后的 `hand_eye_calibration_node` 支持两种采集模式：

### 模式1：手动采集（通过 `/tool_pose` 话题）
- 监听机械臂实时位姿话题
- 用户需要手动移动机械臂到不同位置
- 节点自动检测棋盘并保存样本
- **适用场景**：需要精确控制采集位置，或验证特定位置

### 模式2：自动采集（通过 `/target_pose` 话题）✨ NEW
- 订阅目标位姿话题 `/target_pose` (Pose 类型，不含时间戳)
- 当接收到新位姿时，自动使用该位姿作为采集目标
- 节点自动进行棋盘检测和样本保存
- 完整收集15个样本后自动触发标定计算
- **适用场景**：自动化批量采集，脚本化标定流程

## 快速开始

### 步骤1：启动标定节点

```bash
# 终端1：启动标定节点
source /home/arnoyin/grad_proj/other_hands/implementation/arm_ws/install/setup.bash
ros2 run hand_eye_calibration hand_eye_calibration_node
```

预期输出：
```
[INFO] [hand_eye_calibration_node]: Starting Hand-Eye Calibration Node...
[INFO] [hand_eye_calibration_node]: min_samples = 15
```

### 步骤2：发送自动采集位姿

#### 方式A：使用自动采集脚本（推荐）

```bash
# 终端2：运行自动采集脚本
source /home/arnoyin/grad_proj/other_hands/implementation/arm_ws/install/setup.bash
python3 src/hand_eye_calibration/scripts/auto_collect_example.py
```

脚本会自动：
- 每3秒发送一个预定义的标定位姿
- 依次发送全部15个位姿
- 显示进度：`[1/15] 发送位姿: [-0.05, -0.05, 0.30]`

#### 方式B：手动发送单个位姿

```bash
# 终端2：手动发送位姿
ros2 topic pub --once /target_pose geometry_msgs/Pose \
  "{position: {x: -0.05, y: 0.0, z: 0.35}, \
    orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}"
```

### 步骤3：观察采集进度

标定节点会显示采集进度：

```
[INFO] Target pose received: [-0.05, -0.05, 0.30]
[AutoCollection 1/15] Position: [-0.05, -0.05, 0.30]
[AutoCollection 2/15] Position: [-0.05, 0.00, 0.35]
...
[AutoCollection 15/15] Position: [-0.00, 0.00, 0.35]
[INFO] Computing calibration with 15 samples...
[INFO] Calibration computed successfully!
```

## 代码改动详解

### 1. 新增成员变量
```cpp
geometry_msgs::msg::Pose::SharedPtr latest_target_pose_;          // 存储最新的目标位姿
rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub_;  // /target_pose 订阅
bool using_target_pose_ = false;  // 标志位：是否处于自动采集模式
```

### 2. 新增 /target_pose 订阅
```cpp
target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "/target_pose", 10,
    std::bind(&HandEyeCalibrationNode::targetPoseCallback, this, ...));
```

**为什么用 `Pose` 而不是 `PoseStamped`？**
- `Pose`：不含时间戳，轻量级，适合位置命令
- `PoseStamped`：含时间戳，适合实时反馈

### 3. 新增 targetPoseCallback 方法
```cpp
void targetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    // 将 Pose 转换为 PoseStamped 供后续处理
    auto pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose_stamped->pose = *msg;
    latest_arm_pose_ = pose_stamped;
    
    using_target_pose_ = true;  // 切换到自动采集模式
    // 打印收到的位姿信息
}
```

### 4. 增强同步逻辑
```cpp
// 在 syncTimerCallback() 中
if (using_target_pose_) {
    RCLCPP_INFO(..., "[AutoCollection %zu/%d] Position: ...", 
                 samples_.size(), min_samples_);
} else {
    RCLCPP_INFO(..., "[Sample %zu] Arm position: ...");
}
```

**优点**：
- 自动采集时显示采集进度 X/15
- 手动模式不显示总数（可以采集任意个数）
- 棋盘检测失败时也显示当前进度

## 关键位姿列表

所有位姿使用 D1 机械臂参考方向：
```
位置：    x = -0.05m (适中前伸)
         y ∈ [-0.10, +0.10]m (左右摆动)
         z ∈ [0.30, 0.40]m (上下调整)

方向：    qx = 0.0
         qy = 0.707  (水平向前)
         qz = 0.0
         qw = 0.707
```

### 15个标定位姿分布：
1. **中心区域**（3个）：覆盖主要采集范围
   - [-0.05, -0.05, 0.30]
   - [-0.05, +0.00, 0.35] (参考位置)
   - [-0.05, +0.05, 0.40]

2. **左侧区域**（3个）：Y = -0.10 ~ -0.06
3. **右侧区域**（3个）：Y = +0.06 ~ +0.10
4. **前方区域**（3个）：X = -0.10 ~ -0.06
5. **角落位置**（3个）：覆盖极端区域

**为什么是15个？**
- Tsai-Lenz 算法要求至少 3 个不同位姿，但通常需要 ≥ 15 个以获得鲁棒结果
- 15个位姿能覆盖 1.2m × 0.2m × 0.1m 工作空间
- 避免过多导致采集时间过长

## 故障排查

### 问题1：收不到目标位姿

```bash
# 检查话题发布是否正常
ros2 topic echo /target_pose
```

应该看到定期更新的位姿数据。

### 问题2：棋盘检测失败

输出：`Checkerboard not detected (have X/15 samples)`

**检查项**：
- 棋盘是否在摄像头视野内？
- 光照是否充足？
- 棋盘是否清晰（没有模糊）？

### 问题3：采集卡住（总是显示相同样本数）

**可能原因**：
- 位姿太接近之前的位置（IK无解）
- 摄像头视角遮挡
- 棋盘检测参数不匹配

**解决**：
1. 给足够的移动时间（当前设置3秒/个位姿）
2. 检查机械臂是否成功移动到目标位置
3. 增加棋盘与摄像头的距离

### 问题4：标定计算失败

输出：`Error computing calibration: ...`

**可能原因**：
- 样本数量不足或质量差
- 所有样本点太接近（数值条件数差）
- 棋盘检测不稳定

**解决**：
- 确保所有15个位姿都成功采集
- 检查各位姿的位置差异是否足够大
- 重新执行采集流程

## 输出文件

标定完成后，生成的标定结果保存在：

```
~/hand_eye_calibration_result.yaml
```

内容示例：
```yaml
calibration_result:
  T_base_camera:
    - [R_xx, R_xy, R_xz, tx]
    - [R_yx, R_yy, R_yz, ty]
    - [R_zx, R_zy, R_zz, tz]
    - [0.0, 0.0, 0.0, 1.0]
  reprojection_error: 1.234
```

## 性能优化建议

1. **采集间隔**：当前 3 秒/位姿
   - 增加间隔：给机械臂更多移动时间
   - 减少间隔：加快采集速度（风险：IK失败）

2. **棋盘尺寸**：当前 9×6，30mm 方格
   - 太小：检测困难
   - 太大：视野受限

3. **采集位姿数**：当前 15 个
   - 增加到 20-30：提高鲁棒性（采集时间更长）
   - 减少到 10：加快采集（精度下降风险）

## 后续应用

标定结果可用于：
1. **目标跟踪**：视觉伺服控制（visual_servo_controller_node）
2. **精确抓取**：计算相机坐标系中物体的位置
3. **质量检验**：验证标定质量（reprojection_error < 5px）

---

**有问题？** 查看节点日志了解更多细节：
```bash
ros2 run hand_eye_calibration hand_eye_calibration_node 2>&1 | tee calibration.log
```
