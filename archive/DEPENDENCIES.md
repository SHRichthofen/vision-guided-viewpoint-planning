# Python 依赖分析

## 所需的依赖列表

### 必需（Critical）
- **ultralytics** - YOLO模型加载和推理（vision_detection节点）
- **pyrealsense2** - RealSense D435i相机驱动（vision_detection节点）
- **opencv-python** (cv2) - 图像处理（多个节点）
- **numpy** - 数值计算（所有节点）
- **scipy** - 科学计算，特别是Rotation（vision_arm_control相关节点）
- **PyYAML** (yaml) - 配置文件解析（vision_to_arm_transform节点）

### 可选（Optional）
- rclpy - ROS2 Python客户端库（通常已通过ROS2安装）
- geometry_msgs, std_msgs, sensor_msgs - ROS消息类型（通常已通过ROS2安装）
- ament_index_python - ROS2包索引（通常已通过ROS2安装）

## 安装命令

### 方案1：一次性安装所有
```bash
pip install ultralytics pyrealsense2 opencv-python numpy scipy PyYAML
```

### 方案2：分开安装（推荐用于调试）
```bash
pip install numpy scipy PyYAML         # 基础依赖
pip install opencv-python              # 图像处理
pip install ultralytics                 # YOLO模型（可能需要较长时间）
pip install pyrealsense2                # RealSense（可选，仅实际硬件需要）
```

## 版本建议

- ultralytics: >=8.0.0
- pyrealsense2: >=2.50.0
- opencv-python: >=4.5.0
- numpy: >=1.19.0
- scipy: >=1.5.0
- PyYAML: >=5.3

## 当前节点依赖情况

| 节点 | 关键依赖 | 状态 |
|------|---------|------|
| cylinder_detection | ultralytics, pyrealsense2, cv2, numpy | ✅ 已安装 |
| vision_to_arm_transform | scipy, numpy, yaml | ✅ 已安装 |
| arm_pose_controller | scipy, numpy | ✅ 已安装 |
| target_selector | cv2, numpy | ✅ 已安装 |

## 已安装的版本

```
opencv-python: 4.13.0.92
pyrealsense2: 2.56.5.9235
ultralytics: 8.4.26
numpy: 2.2.6
scipy: 1.15.3
PyYAML: 5.4.1
```

## 系统状态

✅ **系统完全就绪** - 所有4个节点可以同时启动！

```bash
ros2 launch vision_arm_control vision_arm_integration.launch.py
```

### 警告信息（可忽略）
- Qt字体缺失（target_selector UI显示正常，仅样式警告）
- CUDA驱动过旧（仅涉及GPU加速，CPU推理工作正常）
