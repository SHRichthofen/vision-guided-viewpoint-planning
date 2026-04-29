# D435i 相机参数快速参考卡

## 📸 D435i RGB 相机核心参数

### **相机内参矩阵 (K Matrix)**

```
┌─────────────┬─────────┬──────────┐
│  fx=906.948 │    0    │ cx=648.4 │
├─────────────┼─────────┼──────────┤
│      0      │ fy=905.9│ cy=383.9 │
├─────────────┼─────────┼──────────┤
│      0      │    0    │    1     │
└─────────────┴─────────┴──────────┘
```

### **参数含义**
| 参数 | 值 | 单位 | 含义 |
|-----|-----|------|------|
| fx, fy | 906, 906 | 像素 | 焦距 |
| cx, cy | 648, 384 | 像素 | 主点（图像中心） |
| 分辨率 | 1280×720 | 像素 | RGB分辨率 |
| 畸变 | [0,0,0,0,0] | - | **无畸变** |

### **快速计算**

**像素坐标 → 相机坐标（已知深度Z）**

```cpp
// 输入：像素 (u, v)，深度 Z (米)
double u = 640, v = 360, Z = 0.5;

// D435i 参数
double fx = 906.948, fy = 905.906;
double cx = 648.379, cy = 383.873;

// 输出：相机坐标系中的 (X, Y, Z)
double X = (u - cx) * Z / fx;     // = (640 - 648.379) * 0.5 / 906.948
double Y = (v - cy) * Z / fy;     // = (360 - 383.873) * 0.5 / 905.906
// Z 保持不变

// 结果
X ≈ -0.0046 m
Y ≈ -0.0132 m
Z = 0.5 m
```

---

## 🎯 标定文件配置

### **hand_eye_calibration.yaml**
```yaml
camera_fx: 906.948
camera_fy: 905.906
camera_cx: 648.379
camera_cy: 383.873
```

### **Python 使用**
```python
K = np.array([
    [906.948, 0, 648.379],
    [0, 905.906, 383.873],
    [0, 0, 1]
])

dist_coeffs = np.zeros(5)  # D435i 已校正
```

### **C++ 使用**
```cpp
cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
K.at<double>(0, 0) = 906.948;  // fx
K.at<double>(1, 1) = 905.906;  // fy
K.at<double>(0, 2) = 648.379;  // cx
K.at<double>(1, 2) = 383.873;  // cy

cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
```

---

## 🔧 OpenCV 常用操作

### **1. PnP 求解（棋盘位姿）**
```cpp
std::vector<cv::Point3f> object_points;
// ... 填充棋盘3D点 ...

std::vector<cv::Point2f> image_points;
// ... 填充图像中的2D角点 ...

cv::Mat rvec, tvec;
cv::solvePnP(object_points, image_points, K, dist_coeffs, 
             rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

// tvec 是相机到棋盘的平移 (3x1)
// rvec 是旋转向量 (3x1)

// 转换为旋转矩阵
cv::Mat R;
cv::Rodrigues(rvec, R);
```

### **2. 像素去畸变**
```cpp
std::vector<cv::Point2f> distorted_pts, undistorted_pts;
// ... 填充 distorted_pts ...

// D435i 无需去畸变（dist_coeffs = [0, 0, 0, 0, 0]）
// 但为完整性可以调用
cv::undistortPoints(distorted_pts, undistorted_pts, K, dist_coeffs);
```

### **3. 棋盘检测**
```cpp
cv::Mat gray;
cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

std::vector<cv::Point2f> corners;
bool found = cv::findChessboardCorners(
    gray, cv::Size(9, 6), corners,
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

if (found) {
    // 细化角点位置
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
}
```

---

## 📊 关键指标对比

| 相机 | fx | fy | cx | cy | 分辨率 | 畸变 |
|-----|-----|-----|-----|-----|--------|------|
| D435i RGB | 906.9 | 905.9 | 648.4 | 383.9 | 1280×720 | 无 |
| D435 RGB | 920.0 | 921.0 | 638.0 | 366.0 | 1280×720 | 小 |
| 典型USB相机 | 600-800 | 600-800 | 320-640 | 240-480 | 640×480 | 有 |

---

## ⚠️ 常见错误

### ❌ 错误用法
```cpp
// 错误1：用错焦距
double wrong_fx = 615.0;  // ← 这是假设值，D435i 是 906.948

// 错误2：忘记畸变系数
cv::solvePnP(..., rvec, tvec);  // ← 未传 dist_coeffs

// 错误3：混淆焦距单位
// fx 是 **像素** 不是 mm！
```

### ✅ 正确用法
```cpp
// 正确1：使用实际参数
double fx = 906.948;  // D435i 实际值

// 正确2：传递畸变参数
cv::solvePnP(obj_pts, img_pts, K, dist_coeffs, rvec, tvec);

// 正确3：理解焦距单位
// 实际焦距(mm) = fx(像素) × 像素尺寸(mm/像素)
// 对 D435i：906 × 0.003 ≈ 2.7mm
```

---

## 🧮 数学公式速查

### **透视投影方程**
$$
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \frac{1}{Z} \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} X \\ Y \\ Z \end{bmatrix}
$$

### **反解（已知Z）**
$$
X = \frac{(u - c_x) \cdot Z}{f_x}, \quad Y = \frac{(v - c_y) \cdot Z}{f_y}
$$

### **焦距（毫米）**
$$
f_{mm} = f_{pixel} \times s_{pixel}
$$

其中 $s_{pixel}$ 是像素尺寸（通常 2-5 μm）

---

## 🚀 一分钟上手

1. **复制参数到代码**：
   ```cpp
   double fx = 906.948, fy = 905.906, cx = 648.379, cy = 383.873;
   ```

2. **棋盘检测**：
   ```cpp
   cv::findChessboardCorners(gray, {9, 6}, corners);
   ```

3. **PnP 求解**：
   ```cpp
   cv::solvePnP(board_3d, corners_2d, K, dist_coeffs, rvec, tvec);
   ```

4. **获取末端位姿**：从机械臂获取当前位置

5. **进行标定**：运行 `hand_eye_calibration_node`

6. **保存结果**：标定完成，得到 $T_{tool}^{cam}$

7. **应用于视觉伺服**：使用标定结果进行视觉控制

---

## 📚 需要更多帮助？

- 查看：[D435i_camera_params_guide.md](D435i_camera_params_guide.md)
- 标定指南：[hand_eye_calibration_guide.md](hand_eye_calibration_guide.md)
- 代码示例：`hand_eye_calibration.cpp`, `visual_servo_controller.cpp`

---

**🎯 记住最关键的三个数字：906.9, 905.9, (648.4, 383.9)**
