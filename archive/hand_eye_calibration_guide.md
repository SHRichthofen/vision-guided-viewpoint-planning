# 手眼标定完整指南

## 📌 标定前准备

### 1. **相机内参标定** (若还没有)
```bash
# 使用 OpenCV 标定工具
rosrun camera_calibration_parsers convert_camerinfo_to_ini camera_info.yaml camera_calibration.ini

# 或使用标准标定工具包
sudo apt install ros-humble-camera-calibration
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/camera/image_raw camera:=/camera --no-service-check
```

### 2. **准备棋盘**
- 打印棋盘（建议 A3 大小）
- 棋盘应该平整，贴在刚性表面上
- 建议规格：8x6 或 9x7，每格 30mm

### 3. **相机安装**
- 将相机安装在机械臂末端执行器上
- 确保相机视野能看到整个棋盘
- 记录相机相对于末端工具的初始位置（粗略）

---

## 🚀 标定执行步骤

### 步骤 1: 修改配置文件
编辑 `src/control/config/hand_eye_calibration.yaml`，填入相机内参：

```yaml
# 从相机标定结果中获取 fx, fy, cx, cy
camera_fx: 615.0              # 从标定结果获取
camera_fy: 615.0
camera_cx: 320.0
camera_cy: 240.0
```

### 步骤 2: 启动相机节点
```bash
# 例如使用 usb_cam
roslaunch usb_cam usb_cam-test.launch camera_name:=camera device:=/dev/video0
```

### 步骤 3: 启动标定节点
```bash
# 终端 1: 启动 pose_mover（控制机械臂）
ros2 run control pose_mover_node

# 终端 2: 启动手眼标定
ros2 launch control hand_eye_calibration.launch.py
```

### 步骤 4: 采集标定数据

手动或通过手柄控制机械臂，使其**移动到不同位置**（至少 10 个）。
要求：
- 相机始终能清晰看到棋盘
- 棋盘在不同方向和距离
- 每个位置停留 1-2 秒，让系统捕获

**关键**：采集位置应该**充分分散**，覆盖工作空间的不同区域

#### 建议采集位置：
```
位置 1-3: 棋盘在相机正前方，不同距离
位置 4-6: 棋盘相对于相机倾斜 30°-60°
位置 7-9: 棋盘在相机左/右侧
位置 10+: 棋盘在上/下方向
```

### 步骤 5: 自动计算标定

当采集 10+ 个样本时，节点会自动：
1. **检测棋盘角点**（OpenCV `findChessboardCorners`）
2. **PnP 求解**（`solvePnP`）计算相机看到的棋盘位姿
3. **Tsai-Lenz 算法**求解末端 → 相机变换
4. **验证误差**（重投影误差）

查看输出日志：
```
[hand_eye_calibration_node] Computing calibration with 15 samples...
[hand_eye_calibration_node] Calibration completed! Reprojection error: 0.000234 meters
[hand_eye_calibration_node] Tool → Camera Transform:
  Translation: [0.0523, -0.0045, 0.1234] m
  Rotation (quat): [0.0012, 0.7071, -0.0015, 0.7070]
[hand_eye_calibration_node] Calibration result saved to /tmp/hand_eye_calibration_result.yaml
```

---

## 📊 验证标定结果

### 1. **检查重投影误差**
- **好的标定**：误差 < 0.01 m (1 cm)
- **可接受**：误差 < 0.05 m (5 cm)
- **需要重新标定**：误差 > 0.1 m

### 2. **视觉验证**
使用标定结果进行视觉伺服测试：
```cpp
// 使用标定得到的 T_tool_cam
Eigen::Isometry3d T_tool_cam;  // 从 YAML 加载

// 在视觉控制中使用
Eigen::Isometry3d T_base_cam = T_base_tool * T_tool_cam;
// 基于相机观测调节机械臂...
```

### 3. **多次标定验证**
建议进行 2-3 次独立标定，对比结果的一致性：
```bash
# 每次标定后清除旧数据
ros2 service call /reset_calibration std_srvs/srv/Empty {}
```

---

## 🔧 数学原理

### **Tsai-Lenz 标定方程**

给定 $n$ 个采样点，每个点有：
- 机械臂末端位姿：$T_{base}^{tool}[i]$
- 相机观测到的棋盘位姿：$T_{cam}^{board}[i]$

求解末端到相机的变换 $X = T_{tool}^{cam}$，使得：

$$A[i] \cdot X = X \cdot B[i] \quad \forall i$$

其中：
- $A[i] = T_{base}^{tool}[i]$（机械臂运动）
- $B[i] = T_{cam}^{board}[i]$（棋盘相对相机固定）

### **求解步骤**

1. **旋转部分**：分离旋转矩阵 $R$ 的方程
   $$A_R[i] \cdot R = R \cdot B_R[i]$$
   使用 SVD 分解求解

2. **平移部分**：已知旋转后求平移
   $$(I - B_R[i]) \cdot t = A_t[i] - R \cdot B_t[i]$$
   构建超定方程组，使用最小二乘法求解

---

## ❌ 常见问题排查

| 问题 | 原因 | 解决方案 |
|-----|-----|--------|
| 棋盘检测失败 | 光线不足/角度不佳 | 改善光线、调整相机角度 |
| 重投影误差很大 | 采集位置分布不均 | 增加采集数量、覆盖更多方向 |
| 相机内参错误 | 使用了错误的相机标定 | 重新标定相机内参 |
| PnP 求解失败 | 棋盘太小/相机视野有限 | 使用更大的棋盘 |

---

## 📁 文件说明

```
src/control/
├── src/
│   ├── hand_eye_calibration.cpp      ← 标定节点实现
│   └── pose_mover.cpp                ← 修改后的机械臂控制节点
├── launch/
│   └── hand_eye_calibration.launch.py ← 启动脚本
└── config/
    └── hand_eye_calibration.yaml     ← 标定参数配置
```

---

## 💾 保存和加载标定结果

### 保存格式
```yaml
# /tmp/hand_eye_calibration_result.yaml
camera_to_tool:
  translation:
    x: 0.0523
    y: -0.0045
    z: 0.1234
  rotation:
    x: 0.0012
    y: 0.7071
    z: -0.0015
    w: 0.7070
```

### 在应用中加载
```cpp
#include <yaml-cpp/yaml.h>

YAML::Node config = YAML::LoadFile("/tmp/hand_eye_calibration_result.yaml");
double tx = config["camera_to_tool"]["translation"]["x"].as<double>();
double ty = config["camera_to_tool"]["translation"]["y"].as<double>();
double tz = config["camera_to_tool"]["translation"]["z"].as<double>();

double qx = config["camera_to_tool"]["rotation"]["x"].as<double>();
double qy = config["camera_to_tool"]["rotation"]["y"].as<double>();
double qz = config["camera_to_tool"]["rotation"]["z"].as<double>();
double qw = config["camera_to_tool"]["rotation"]["w"].as<double>();

Eigen::Isometry3d T_tool_cam = Eigen::Isometry3d::Identity();
T_tool_cam.translation() = Eigen::Vector3d(tx, ty, tz);
T_tool_cam.rotation() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
```

---

## 🎯 后续应用

有了手眼标定结果 $T_{tool}^{cam}$，你可以：

### 1️⃣ **视觉伺服控制**
```cpp
// 相机捕获目标位置
Eigen::Isometry3d T_cam_target = detectTarget();

// 转换到基座坐标系
Eigen::Isometry3d T_base_target = T_base_tool * T_tool_cam * T_cam_target;

// 移动机械臂到目标
moveToTarget(T_base_target);
```

### 2️⃣ **物体抓取**
```cpp
// 检测物体在相机中的位置
Eigen::Isometry3d T_cam_object = detectObject();

// 计算机械臂应到达的位置
Eigen::Isometry3d T_base_object = T_base_tool * T_tool_cam * T_cam_object;

// 规划并执行抓取
planGraspTrajectory(T_base_object);
```

### 3️⃣ **视觉反馈调整**
```cpp
// 执行初步移动
moveToApproxPosition(target);

// 用视觉反馈微调
while (!atTarget()) {
    Eigen::Isometry3d T_cam_error = measureVisualError();
    Eigen::Isometry3d correction = T_tool_cam.inverse() * T_cam_error * T_tool_cam;
    adjustPosition(correction);
}
```

祝标定顺利！🎉
