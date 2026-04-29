# 圆柱ID 4 投影错位问题诊断

## 观察现象

**左图（实际摄像头）**：
- 圆柱 ID 4 的绿框在正确位置
- 但其红色椭圆圈完全脱离实际管口 → **投影位置错了！**

**右图（调试输出）**：
- 同一图中的椭圆位置正常 → **说明轴比计算本身可能是对的**

**坐标轴**：
- 完全看不到（应该是红、绿、蓝的箭头）

---

## 根本问题分析

这不是深度约束的问题，而是**更上游的问题**！

### 问题链：

```
Real world position (correct) 
        ↓
YOLO detection (mask)
        ↓
Rim pipeline (ellipse fitting)
        ↓
Axis-ratio method → Z, normal (L1)   ← 轴比得到初值
        ↓
Reprojection optimize + depth prior   ← 我们的深度约束
        ↓
Final pose (L2)
        ↓
draw_pose_info() 投影回图像 (L3)      ← 这里出错了！
        ↓
Left image: Red ellipse in wrong place

```

### 三个可能的失败点

#### 问题 1️⃣：L1（轴比初值）本身错误

**症状**：
- Z 值计算错（例如应该 0.3m，算出 0.5m）
- 法向量（normal）方向错

**原因**：
- 椭圆长短轴比错
- 梅比乌斯环问题（法向反向）

#### 问题 2️⃣：L2（优化后）被约束拉错

**症状**：
- L1 是对的，但深度约束强制往错的方向拉

**检查**：
```python
# 深度约束说的是：不要离初值太远
z_prior = z0  # 初值
depth_residual = (z - z_prior)² / σ²

# 如果 z0 本身就错了，约束反而锁定错误！
```

**例子**：
- 真实 Z = 0.30m
- 轴比算出 Z = 0.50m（错）
- 优化开始：Z 想优化到 0.28m，但约束说"别离 0.50m 太远"
- 最终：Z ≈ 0.48m（还是错，被锁死了）

#### 问题 3️⃣：L3（投影回图像）坐标系不一致

**症状**：
- 位姿本身对，但投影时用了错的相机内参或坐标系

**检查**：
```python
# project_point_3d_to_2d 的公式
u = fx * X/Z + ppx
v = fy * Y/Z + ppy

# 如果这里的 X, Y, Z 不是在相机坐标系中，就全错了！
```

---

## 即时诊断步骤

### Step 1：启用详细日志

在 `detection_node.py` 的 `callback_image()` 中添加诊断输出：

```python
if live_pose is not None and is_candidate:
    c3d = live_pose['center_3d']
    normal = live_pose['normal']
    
    # L1: 轴比初值
    self.get_logger().info(
        f"L1 Axis-ratio: pos=({c3d[0]:.3f}, {c3d[1]:.3f}, {c3d[2]:.3f})m, "
        f"normal=({normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f})"
    )
    
    # L2: 优化后（检查 optimized 标记）
    if live_pose.get('optimized', False):
        self.get_logger().info(f"L2 Optimized: reprojection_error={live_pose.get('reprojection_error', '?')}")
    
    # L3: 投影验证
    p_center = self.project_point_3d_to_2d(c3d)
    self.get_logger().info(f"L3 Projection: 3D({c3d[2]:.3f}m) → 2D({p_center})")
```

### Step 2：验证椭圆与位姿一致性

在 `draw_pose_info()` 中加入验证：

```python
# 检查椭圆中心和投影中心是否接近
ellipse_center = pose['ellipse_2d'][:2]  # (u, v)
projected_center = self.project_point_3d_to_2d(c_3d)  # (u', v')

distance = np.linalg.norm(
    np.array(ellipse_center) - np.array(projected_center)
)

if distance > 20:  # 超过 20 像素
    self.get_logger().warn(
        f"投影不一致: 椭圆中心 {ellipse_center}, "
        f"计算投影 {projected_center}, 距离 {distance:.1f}px"
    )
```

### Step 3：坐标轴调试

添加坐标轴可见性检查：

```python
# 在 draw_pose_info 中
if cfg.DRAW_AXES and p_center is not None:
    # 绘制坐标轴
    cv2.circle(img, p_center, 4, (255, 255, 255), -1)  # 白点标记中心
    
    # 调试输出
    self.get_logger().debug(
        f"Draw axes at {p_center}: "
        f"p_z={p_z}, p_x={p_x}, p_y={p_y}"
    )
```

---

## 最可能的原因排序

1. **最可能（70%）**：轴比初值 Z 或 normal 本身就错
   - 症状：左右都显示同样的错误位置
   - 测试：关闭深度约束（λ=0），看优化能否自我纠正

2. **次可能（20%）**：深度约束权重过大（λ=0.1太强）
   - 症状：优化无效，完全被锁在初值附近
   - 测试：改为 λ=0.01，重新优化

3. **可能（10%）**：相机内参错误
   - 症状：投影方向完全反
   - 测试：检查 `self.intr.fx, fy, ppx, ppy` 的值

---

## 快速测试方案

### 方案 A：禁用深度约束，看优化是否工作

```python
# 在 config.py 改为
REPROJECTION_USE_DEPTH_PRIOR = False  # 临时禁用

# 重新运行，观察：
# - 红色椭圆是否能自动修正位置？
# - 坐标轴是否出现？
```

**预期结果**：
- ✅ 如果红椭圆修正到正确位置 → 说明优化有效，问题是深度约束
- ❌ 如果红椭圆仍然错位 → 说明轴比初值本身错，或坐标系问题

### 方案 B：降低深度约束权重

```python
# 改为
REPROJECTION_DEPTH_PRIOR_WEIGHT = 0.01  # 降低 10 倍

# 观察优化能否有更大自由度纠正
```

### 方案 C：打印相机内参和位姿

```python
# 在初始化时
self.get_logger().info(f"Camera intrinsics: fx={self.intr.fx}, fy={self.intr.fy}, "
                       f"ppx={self.intr.ppx}, ppy={self.intr.ppy}")

# 在每帧
self.get_logger().info(f"Pose frame {frame_id}: center={live_pose['center_3d']}, "
                       f"normal={live_pose['normal']}")
```

---

## 坐标轴完全看不到的原因

**可能原因**：

1. `project_point_3d_to_2d()` 返回 None
   - Z ≤ 1e-6 → 点在相机后面或太近
   - u 或 v 超出画面边界

2. `cfg.DRAW_AXES = False`（已验证为 True）

3. 坐标轴太短（axis_len = 0.08m）且位置计算错

**修复建议**：

```python
# 在 draw_pose_info 中添加调试
if p_center is None:
    self.get_logger().warn(f"center_3d 投影失败: {c_3d}")
    return

# 强制画一个调试十字
cv2.line(img, (p_center[0]-20, p_center[1]), (p_center[0]+20, p_center[1]), (255,255,255), 1)
cv2.line(img, (p_center[0], p_center[1]-20), (p_center[0], p_center[1]+20), (255,255,255), 1)
cv2.circle(img, p_center, 3, (255, 0, 0), -1)
```

---

## 建议的调试流程

1. **关闭深度约束** → 看优化能否自救
2. **打印所有位姿参数** → 逐帧对比L1/L2/L3
3. **绘制强制白色十字** → 确认投影点位置
4. **验证椭圆-投影对齐** → 应该重合或接近
5. **调整深度权重** → 在 0.01 - 0.3 之间寻找最优值

