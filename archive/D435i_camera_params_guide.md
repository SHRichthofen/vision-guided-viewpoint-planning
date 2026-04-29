# Intel RealSense D435i 相机参数详解

## 📋 参数结构（CameraInfo 格式）

你获取的是标准的 **ROS CameraInfo** 消息格式，结构如下：

```
camera_info
├── header            # 时间戳和坐标系信息
├── height: 720       # 图像高度
├── width: 1280       # 图像宽度
├── distortion_model  # 畸变模型
├── d[]               # 畸变系数 (5个)
├── k[]               # 内参矩阵 (3x3=9个)
├── r[]               # 旋转矩阵 (3x3=9个)
├── p[]               # 投影矩阵 (3x4=12个)
└── roi               # 感兴趣区域
```

---

## 🔍 D435i 参数解读

### **1. 基本信息**
```yaml
height: 720           # 图像高度
width: 1280           # 图像宽度
frame_id: camera_color_optical_frame
distortion_model: plumb_bob
```

✅ **分辨率**：1280×720（标准HD分辨率）
✅ **坐标系**：RGB光学坐标系（Z向前，Y向下，X向右）

---

### **2. K 矩阵（相机内参矩阵）** 🎯

```python
K = [ fx   0  cx ]   =  [ 906.948   0    648.379 ]
    [  0  fy  cy ]       [   0   905.906 383.873 ]
    [  0   0   1 ]       [   0      0       1    ]
```

| 参数 | 值 | 含义 |
|-----|-----|------|
| **fx** | 906.948 | X方向焦距（像素） |
| **fy** | 905.906 | Y方向焦距（像素） |
| **cx** | 648.379 | 主点X坐标（像素） |
| **cy** | 383.873 | 主点Y坐标（像素） |

**实际焦距计算**（如果已知传感器像素尺寸）：
```
f_mm = f_pixel × pixel_size
```

**D435i RGB 传感器**：
- 对角线尺寸：约 1/3 英寸
- 像素尺寸：约 2.75-3.0 μm
- 实际焦距：≈ 906 × 0.003 ≈ **2.7mm**

---

### **3. 主点位置分析** 📍

```
图像中心理论值：(640, 360)
实际主点位置：  (648.379, 383.873)
偏差：          (+8.379, +23.873) 像素
```

**判断**：
- **Δx = +8px**：主点向右偏移（略微）
- **Δy = +24px**：主点向下偏移（常见）
- 偏差很小，说明相机镜头质量较好

---

### **4. D 向量（畸变系数）** ✨

```yaml
d: [0.0, 0.0, 0.0, 0.0, 0.0]
```

**含义**：
```
[k1, k2, p1, p2, k3]
 ├─ k1, k2: 径向畸变系数（桶形/枕形）
 ├─ p1, p2: 切向畸变系数
 └─ k3: 二阶径向畸变系数
```

✅ **全0 = 无畸变**

这意味着：
1. D435i 的RGB图像已被硬件校正
2. 或者该数据是从校正后的图像提取的
3. 可以**直接使用**，无需额外畸变校正

> 💡 **建议**：在 PnP 求解时仍然传入 dist_coeffs（全0），保持完整的数据流

---

### **5. R 矩阵（旋转矩阵）**

```
R = [ 1  0  0 ]
    [ 0  1  0 ]
    [ 0  0  1 ]
```

单位矩阵 = **无旋转校正**

（立体视觉中用于校正左右相机间的旋转，单目相机默认为单位矩阵）

---

### **6. P 矩阵（投影矩阵）**

```
P = [ 906.948   0    648.379   0 ]
    [   0     905.906 383.873  0 ]
    [   0       0       1      0 ]
```

公式：P = [K | 0]（对于单目相机，右侧补 0）

用途：
- 立体视觉中的视图校正
- 单目相机：直接用 K 矩阵

---

## 🧮 坐标转换公式

### **像素坐标 → 相机坐标系**

已知：
- 像素坐标 (u, v)
- 深度 Z_cam（米）

求：三维点在相机坐标系中的坐标 (X, Y, Z)

**公式**：
```
X_cam = (u - cx) × Z_cam / fx
Y_cam = (v - cy) × Z_cam / fy
Z_cam = Z_cam
```

**Python 示例**：
```python
import numpy as np

# D435i 参数
fx, fy = 906.948, 905.906
cx, cy = 648.379, 383.873

# 像素坐标（图像中心）
u, v = 640, 360

# 深度 0.5m
Z = 0.5

# 计算相机坐标
X = (u - cx) * Z / fx  # (640 - 648.379) * 0.5 / 906.948 ≈ -0.0046
Y = (v - cy) * Z / fy  # (360 - 383.873) * 0.5 / 905.906 ≈ -0.0132
Z_cam = Z              # 0.5

print(f"相机坐标: [{X:.4f}, {Y:.4f}, {Z_cam:.4f}] m")
# 输出：相机坐标: [-0.0046, -0.0132, 0.5000] m
```

### **相机坐标系 → 世界坐标系**

```python
# 已知相机在世界中的位姿
T_world_cam = ...  # 4x4 变换矩阵

# 点在相机坐标系中
P_cam = np.array([X, Y, Z, 1])

# 转换到世界坐标系
P_world = T_world_cam @ P_cam
```

---

## 🎬 实际应用示例

### **场景1：检测图像中棋盘的3D位置**

```python
import cv2
import numpy as np

# 相机参数
K = np.array([
    [906.948, 0, 648.379],
    [0, 905.906, 383.873],
    [0, 0, 1]
], dtype=float)

dist_coeffs = np.zeros(5)  # D435i 无畸变

# 读取图像和深度
img = cv2.imread('checkerboard.png')
depth = cv2.imread('depth.png', cv2.IMREAD_UNCHANGED)

# 检测棋盘角点
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

if ret:
    # 细化角点
    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                     (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.001))
    
    # 获取第一个角点的深度
    u, v = int(corners[0, 0, 0]), int(corners[0, 0, 1])
    Z = depth[v, u] / 1000.0  # 深度转米
    
    # 转换为相机坐标
    X = (u - 648.379) * Z / 906.948
    Y = (v - 383.873) * Z / 905.906
    
    print(f"棋盘第一个角点（相机坐标）: [{X:.3f}, {Y:.3f}, {Z:.3f}] m")
```

### **场景2：使用 PnP 求解棋盘位姿**

```python
# 棋盘的3D坐标（棋盘坐标系）
objpoints = []
for i in range(6):
    for j in range(9):
        objpoints.append([j * 0.03, i * 0.03, 0])  # 30mm 棋盘

objpoints = np.array(objpoints, dtype=np.float32)

# PnP 求解
success, rvec, tvec = cv2.solvePnP(objpoints, corners, K, dist_coeffs)

if success:
    # tvec 是相机到棋盘原点的平移（米）
    print(f"棋盘位置（相机坐标）: {tvec.T}")
    
    # 旋转向量转旋转矩阵
    R, _ = cv2.Rodrigues(rvec)
    
    # 构建变换矩阵
    T_cam_board = np.eye(4)
    T_cam_board[:3, :3] = R
    T_cam_board[:3, 3] = tvec.ravel()
    print(f"棋盘位姿（变换矩阵）:\n{T_cam_board}")
```

---

## ⚙️ 使用D435i RGB参数的注意事项

| 项目 | 说明 |
|-----|------|
| **分辨率** | 1280×720（标准，其他分辨率参数会不同） |
| **帧率** | RGB: 30/60fps（深度独立） |
| **畸变** | 已校正（无需额外处理） |
| **立体基线** | RGB 无立体基线（D435i 有 IR 立体） |
| **有效范围** | 0.1 - 10m（实际 0.5 - 3m 效果最好） |
| **FOV** | 约 70° 水平 × 53° 竖直 |

---

## 🔧 配置文件示例

**hand_eye_calibration.yaml**：
```yaml
# 棋盘
checkerboard_rows: 6
checkerboard_cols: 9
square_size_mm: 30.0

# D435i RGB 相机参数
camera_fx: 906.948
camera_fy: 905.906
camera_cx: 648.379
camera_cy: 383.873
camera_width: 1280
camera_height: 720

# 畸变（D435i RGB 已校正）
distortion_model: plumb_bob
d_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]
```

---

## 🎯 常见问题

**Q：为什么 fx ≠ fy？**  
A：传感器像素通常是方形的（1:1），但镜头可能有轻微差异。差异 < 1% 属于正常。

**Q：主点为何不在图像中心？**  
A：镜头光学中心可能不完全对齐传感器中心。这是正常的偏差。

**Q：D435i 没有畸变参数，是否需要校正？**  
A：不需要。RGB图像已被硬件/固件校正。可将 dist_coeffs 设为 0。

**Q：如何在其他分辨率下使用这些参数？**  
A：需要重新标定或使用缩放因子：
```
K_new = K_old × (new_res / old_res)
```

---

## 📚 参考资源

- [Intel RealSense D435i 规格](https://www.intelrealsense.com/depth-camera-d435i/)
- [ROS CameraInfo 说明](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)
- [OpenCV 相机标定](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- [Tsai-Lenz 手眼标定](https://www.kinematics.com/)
