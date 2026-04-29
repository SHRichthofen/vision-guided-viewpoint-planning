# 手眼标定自动采集 - 系统架构图

## 数据流图

```
┌─────────────────────────────────────────────────────────────────┐
│                    自动采集工作流程                              │
└─────────────────────────────────────────────────────────────────┘

     auto_collect_example.py
            (Python脚本)
                  │
                  ├─ 初始化15个标定位姿
                  │  Position: [-0.05, ±0.05, 0.30-0.40]
                  │  Orientation: 水平向前
                  │
                  └─> 每3秒发送一个位姿
                         │
                         ▼
                    /target_pose
                  (geometry_msgs/Pose)
                         │
                         │
        ╔════════════════╩═════════════════╗
        │                                  │
        ▼                                  ▼
   ┌─────────────┐          ┌──────────────────────┐
   │ 摄像头节点   │          │ 手眼标定节点           │
   │(外部系统)   │          │(hand_eye_calibration)│
   └─────────────┘          └──────────────────────┘
        │                            │
        │ /camera/color/image_raw    │
        │                            │ /target_pose
        │  (sensor_msgs/Image)       │ (geometry_msgs/Pose)
        │                            │
        └───────────────┬────────────┘
                        │
                        ▼
        ┌───────────────────────────┐
        │ targetPoseCallback()      │
        │ - 接收位姿命令            │
        │ - 转换为 PoseStamped     │
        │ - 设置 using_target_pose_│
        └───────────────────────────┘
                        │
                        ▼
        ┌───────────────────────────┐
        │ syncTimerCallback()       │
        │ (每100ms触发)            │
        │ - 检测棋盘               │
        │ - 收集样本               │
        │ - 显示进度 X/15          │
        └───────────────────────────┘
                        │
                        ▼
        样本是否 ≥ 15个？
           YES ▼ NO
           │    │
           │    └──> 继续等待下一个位姿
           │
           ▼
    ┌──────────────────────┐
    │ computeCalibration() │
    │ (Tsai-Lenz 算法)     │
    │ - SVD 分解          │
    │ - 求解 A*X=X*B      │
    │ - 计算误差          │
    └──────────────────────┘
           │
           ▼
    calibration_result.yaml
    (标定结果)
```

---

## 节点订阅发布关系

```
┌──────────────────────────────────────────────────────────────┐
│         ROS2 Humble 中间件                                   │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│   发布方                  话题                  订阅方         │
│   ────────────────────────────────────────────────────     │
│                                                              │
│   Camera Driver    ──> /camera/color/image_raw ──> CalibNode
│   (外部ROS节点)         (sensor_msgs/Image)       │        │
│                                                  ▼        │
│   auto_collect.py  ──> /target_pose ──────────────────────┘
│   (Python脚本)         (geometry_msgs/Pose)
│
│   Arm Driver       ──> /tool_pose
│   (备选)                (geometry_msgs/PoseStamped)
│
│   CalibNode ──────> /calibration_result
│   (本节点)          (geometry_msgs/TransformStamped)
│                     + ~/hand_eye_calibration_result.yaml
│
└──────────────────────────────────────────────────────────────┘
```

---

## 代码模块结构

```
hand_eye_calibration_node.cpp
├── HandEyeCalibrationNode : public rclcpp::Node
│   │
│   ├── 构造函数
│   │   ├── 加载参数（棋盘规格、相机内参等）
│   │   ├── 初始化 CheckerboardDetector
│   │   ├── 初始化 TsaiLenzSolver
│   │   ├── 订阅 /target_pose (Pose)             ◄─ 新增
│   │   ├── 订阅 /tool_pose (PoseStamped)        ◄─ 原有
│   │   ├── 订阅 camera_image
│   │   └── 发布 calibration_result
│   │
│   ├── targetPoseCallback()                      ◄─ 新增方法
│   │   ├── 将 Pose 转为 PoseStamped
│   │   ├── 更新 latest_arm_pose_
│   │   ├── 设置 using_target_pose_ = true
│   │   └── 打印接收确认
│   │
│   ├── armPoseCallback()                         ◄─ 保留原有
│   │   ├── 更新 latest_arm_pose_
│   │   └── 设置 using_target_pose_ = false
│   │
│   ├── cameraImageCallback()                     ◄─ 保留原有
│   │   └── 更新 latest_image_
│   │
│   ├── syncTimerCallback()                       ◄─ 增强原有
│   │   ├── 检查数据有效性
│   │   ├── 检测棋盘
│   │   ├── 创建校准样本
│   │   ├── 显示进度
│   │   │   ├── [AutoCollection X/15] (自动模式)
│   │   │   └── [Sample X] (手动模式)
│   │   └── 检查触发条件
│   │
│   ├── computeCalibration()                      ◄─ 保留原有
│   │   ├── 调用 TsaiLenzSolver
│   │   ├── 发布结果
│   │   └── 保存到 YAML 文件
│   │
│   └── poseToIsometry()
│       └── 坐标转换工具
│
├── 成员变量
│   ├── detector_: CheckerboardDetector
│   ├── solver_: TsaiLenzSolver
│   ├── samples_: 校准样本列表
│   ├── latest_arm_pose_: 最新臂位姿
│   ├── latest_image_: 最新图像
│   ├── latest_target_pose_: 最新目标位姿      ◄─ 新增
│   ├── arm_pose_sub_: PoseStamped 订阅
│   ├── target_pose_sub_: Pose 订阅            ◄─ 新增
│   ├── camera_image_sub_: 图像订阅
│   ├── calibration_pub_: 结果发布器
│   ├── sync_timer_: 100ms 定时器
│   ├── min_samples_: 最小样本数 (=15)
│   ├── output_file_: 输出文件路径
│   └── using_target_pose_: 模式标志位          ◄─ 新增
│
└── 全局函数
    └── main(): 节点启动入口
```

---

## 采集流程时序图

```
时间轴      auto_collect.py        hand_eye_calibration_node
──────      ───────────────        ────────────────────────
    T0:     启动脚本
            初始化15个位姿
            │
    T1:     发布位姿1 ──────────────> targetPoseCallback()
            │                         设置位姿1
            │
    T2:                              syncTimerCallback() 
            │                         ├─ 检测棋盘
            │                         ├─ 收集样本1 ✓
            │                         └─ 打印 [AutoCollection 1/15]
            │
    T3+3s:  发布位姿2 ──────────────> targetPoseCallback()
            │                         设置位姿2
            │
    T4:                              syncTimerCallback()
            │                         ├─ 检测棋盘
            │                         ├─ 收集样本2 ✓
            │                         └─ 打印 [AutoCollection 2/15]
            │
    ...     ... (重复13次) ...
            │
    T29+3s: 发布位姿15 ──────────────> targetPoseCallback()
            │                          设置位姿15
            │
    T30:                              syncTimerCallback()
            │                          ├─ 检测棋盘
            │                          ├─ 收集样本15 ✓
            │                          ├─ 打印 [AutoCollection 15/15]
            │                          ├─ samples_.size() >= 15 ✓
            │                          └─ 触发 computeCalibration()
            │
    T31:                              computeCalibration()
            │                          ├─ 调用 TsaiLenzSolver
            │                          ├─ 解算手眼变换
            │                          ├─ 计算重投影误差
            │                          ├─ 发布结果
            │                          ├─ 保存到 YAML
            │                          └─ 打印成功消息
            │
    T32:    脚本显示
            "所有位姿已发送！
            等待标定计算完成..."
            │
            └────────────────────────> 标定完成！✓
```

---

## 15个标定位姿分布可视化

```
从俯视角度 (俯视图):

         Y轴 (左右)
         ↑
    -0.10┼─────────────┐
         │  7  10  13  │
         │             │  样本位置
         │  1  4   8   │
    0    ├  2  5  11   ├───────> X轴 (前后)
         │  3  6  12   │
         │  9  14 15   │
    +0.10│─────────────┘
         
样本范围：
• X: [-0.10, -0.00]m (厘米级精度)
• Y: [-0.10, +0.10]m (左右摆动)
• Z: [ 0.30,  0.40]m (竖直范围)

从侧视角度 (前视图):

         Z轴 (竖直)
         ↑
    0.40 ├─────────────┐
         │  3  6  12   │
         │             │  高度分布
         │  2  5  11   │
    0.35 ├  1  4   8   ├─────> Y轴
         │             │
         │  7  10  13  │
    0.30 ├─────────────┘
         
覆盖范围：竖直方向10cm
```

---

## 消息格式说明

### 输入：/target_pose (geometry_msgs/Pose)
```yaml
position:
  x: -0.05  # 机械臂 X 坐标 (m)
  y:  0.00  # 机械臂 Y 坐标 (m)
  z:  0.35  # 机械臂 Z 坐标 (m)
orientation:
  x:  0.0   # 四元数 (无旋转成分)
  y:  0.707 # 水平向前 (绕Y轴90°)
  z:  0.0   # (无旋转成分)
  w:  0.707 # (无旋转成分)
```

### 输出：hand_eye_calibration_result.yaml
```yaml
calibration_result:
  T_base_camera:
    # 旋转矩阵 + 平移向量
    - [R_xx, R_xy, R_xz, tx]
    - [R_yx, R_yy, R_yz, ty]
    - [R_zx, R_zy, R_zz, tz]
    - [0.0,  0.0,  0.0,  1.0]
  reprojection_error: 1.234  # 像素单位，越小越好
```

---

## 错误处理流程

```
syncTimerCallback()
    │
    ├─ 数据有效性检查
    │   ├─ latest_arm_pose_ == nullptr? ──> 返回
    │   └─ latest_image_ == nullptr? ──────> 返回
    │
    ├─ 图像转换
    │   └─ cv_bridge 转换失败? ────────────> 返回
    │
    ├─ 棋盘检测
    │   ├─ 检测成功?
    │   │   ├─ YES: 继续
    │   │   └─ NO:
    │   │       ├─ 如果 using_target_pose_:
    │   │       │   WARN: "Checkerboard not detected (have X/15 samples)"
    │   │       └─ 否则:
    │   │           WARN: "Checkerboard not detected"
    │   │       返回
    │   │
    │   └─ 样本收集
    │       ├─ 创建 CalibrationSample
    │       ├─ 添加到 samples_ 列表
    │       └─ 显示进度
    │           ├─ 如果 using_target_pose_:
    │           │   INFO: "[AutoCollection X/15] Position: ..."
    │           └─ 否则:
    │               INFO: "[Sample X] Arm position: ..."
    │
    └─ 触发条件检查
        ├─ samples_.size() >= min_samples_? ──> 触发 computeCalibration()
        └─ 否则: 等待更多样本
```

---

## 模式切换逻辑

```
     开机默认
        │
        ▼
   using_target_pose_ = false (手动模式)
        │
        ├─ 接收 /tool_pose ──> armPoseCallback()
        │                     设置 using_target_pose_ = false
        │
        └─ 接收 /target_pose ──> targetPoseCallback()
                               设置 using_target_pose_ = true
                               
切换示意：
  
  手动模式 ◄─────────────────► 自动模式
  /tool_pose                 /target_pose
  显示[Sample X]             显示[AutoCollection X/15]
  无进度指示                 显示X/15进度
```

---

## 性能指标预测

```
┌─────────────────────────────────────────────┐
│  项目            │  时间  │  CPU  │  内存  │
├─────────────────┼────────┼───────┼────────┤
│ 棋盘检测         │  20-50 ms │ 30%  │ 50MB   │
│ PnP 求解         │  1-5 ms   │ 10%  │ 10MB   │
│ 样本存储(1张)    │  <1 ms    │ 5%   │ 5MB    │
│ Tsai-Lenz 解算   │  200-500ms│ 50%  │ 100MB  │
│ 总采集时间(15个) │  ~45 秒   │ -    │ -      │
│ 标定精度         │  -     │ -     │ -      │
│  (重投影误差)    │  1-3 px  │ -    │ -     │
└─────────────────────────────────────────────┘

系统要求：
• CPU: ≥ 2核，1.5GHz
• 内存: ≥ 512MB
• ROS2: Humble 或更新版本
• OpenCV: ≥ 4.5
• Eigen3: ≥ 3.4
```

---

**该架构图更新于**: 2025-03-18
