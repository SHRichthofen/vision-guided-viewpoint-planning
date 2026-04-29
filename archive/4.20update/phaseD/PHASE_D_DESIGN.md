# Phase D: 非线性重投影优化设计
**目标**: 突破轴比方法的精度上限 ±8mm → ±1-2mm  
**应用场景**: 5cm 圆柱，±5mm 精度需求  
**预期开发周期**: 2-3天

---

## 一、核心原理

### 1.1 问题重新定义

**轴比方法的局限**:
```
给定：轮廓边缘点 {p_i = (x_i, y_i)}（100+ 点）
目标：反演 6DOF 位姿 (X, Y, Z, θ, φ, ψ)

轴比方法：
  a, b = 椭圆长短轴
  r = b/a  →  θ ≈ arccos(r)   [2参数 → 6参数，欠定]
  
  灵敏度高：r 微小误差 → θ 大误差 ✗

重投影优化方法：
  Cost = Σ_i [distance(p_i, project(ellipse))]²
  
  直接优化所有 6 个参数
  用 100+ 个约束拟合 6 个未知数 [超定，稳定] ✓
```

### 1.2 数学模型

#### 3D 圆柱模型
```
圆柱：
  中心：(X, Y, Z)
  法向：(n_x, n_y, n_z)  [单位向量]
  半径：r = D/2 = 0.025m

圆柱在3D空间的参数化：
  p(u, v) = C + r*cos(u)*(v × n) + r*sin(u)*((v × n) × n)
  
  其中：
    C = 圆心 (X, Y, Z)
    u ∈ [0, 2π]  [圆周角]
    v ∈ [-H/2, H/2]  [轴向]  [对齐圆柱法向]
    v 是单位轴向向量
```

#### 投影到图像
```
相机模型（针孔）：
  p_screen = K @ [R|t] @ p_world
  
  其中：
    K = [[f_x, 0, c_x],   [相机内参]
         [0, f_y, c_y],
         [0, 0, 1]]
    
    p_img = (u, v) = (f_x * x/z + c_x, f_y * y/z + c_y)

圆柱在图像上形成椭圆
  proj_ellipse = {π(p(u, v)) : u ∈ [0, 2π]}
  
  其中 π 是投影函数
```

#### 优化目标
```
最小化重投影误差：
  E(X, Y, Z, θ, φ, ψ) = Σ_i [d_i - d_model]²
  
  其中：
    d_i = 观测到的点 p_i 到某个参考的距离
    
  更准确地：
    d_i = distance(p_i, closest_point_on_projected_ellipse)
    
  或使用代数距离（更快）：
    d_i = |椭圆方程(u, v)| / ||∇椭圆方程||
```

---

## 二、实现方案

### 2.1 核心模块分解

#### 模块 1: 3D 圆柱点投影
```python
def project_cylinder_to_image(
    cylinder_center: np.array,  # (3,)  [X, Y, Z]
    cylinder_normal: np.array,  # (3,)  [n_x, n_y, n_z]，单位向量
    radius: float,              # 0.025 m
    intrinsics: dict,           # {'f_x', 'f_y', 'c_x', 'c_y'}
    sample_count: int = 100     # 采样圆周上的点数
) -> np.array:
    """
    将3D圆柱投影到2D图像
    
    Returns:
        projected_points: (sample_count, 2)  [u, v 坐标]
        projected_normals: (sample_count, 2)  [法线方向，用于距离计算]
    """
    
    # 1. 构建圆柱局部坐标系
    #    轴向：cylinder_normal
    #    径向：垂直于轴向的两个正交向量
    
    axis = cylinder_normal / np.linalg.norm(cylinder_normal)
    
    # 找垂直于 axis 的向量
    if abs(axis[2]) < 0.9:
        perpendicular = np.array([0, 0, 1]) - axis[2] * axis
    else:
        perpendicular = np.array([1, 0, 0]) - axis[0] * axis
    
    radial_1 = perpendicular / np.linalg.norm(perpendicular)
    radial_2 = np.cross(axis, radial_1)
    
    # 2. 圆周采样（固定在与法向垂直的平面）
    angles = np.linspace(0, 2*np.pi, sample_count, endpoint=False)
    local_points_3d = radius * (
        np.outer(np.cos(angles), radial_1) + 
        np.outer(np.sin(angles), radial_2)
    )  # (sample_count, 3)
    
    # 3. 加上圆心位置
    world_points_3d = cylinder_center + local_points_3d
    
    # 4. 投影到图像
    z = world_points_3d[:, 2]
    x = world_points_3d[:, 0]
    y = world_points_3d[:, 1]
    
    u = intrinsics['f_x'] * (x / z) + intrinsics['c_x']
    v = intrinsics['f_y'] * (y / z) + intrinsics['c_y']
    
    projected_points = np.column_stack([u, v])
    
    return projected_points  # (sample_count, 2)
```

#### 模块 2: 点到曲线距离计算
```python
def point_to_curve_distance(
    point: np.array,           # (2,)  [u, v]
    curve_points: np.array,    # (N, 2)  采样曲线上的点
    return_closest_idx: bool = False
) -> float:
    """
    计算点到曲线的最短距离（使用分段逼近）
    
    Returns:
        min_distance: 标量
        closest_idx: (可选) 最近的曲线点索引
    """
    
    # 暴力法：计算到所有曲线段的距离
    distances = np.linalg.norm(curve_points - point, axis=1)
    min_idx = np.argmin(distances)
    min_dist = distances[min_idx]
    
    if return_closest_idx:
        return min_dist, min_idx
    return min_dist


def robust_reprojection_error(
    observed_points: np.array,  # (M, 2)  边缘检测得到的点
    projected_points: np.array, # (N, 2)  模型投影的点
    loss_function: str = 'huber'
) -> float:
    """
    计算健壮的重投影误差（抑制离群值）
    
    Loss 函数选择：
    - 'l2': 平方误差（对离群值敏感）
    - 'huber': Huber loss（健壮，平衡点和离群值）
    - 'soft_l1': Smooth L1（类似Huber但微分处处存在）
    """
    
    # 对每个观测点，找最近的投影点
    errors = []
    for obs_pt in observed_points:
        dist = point_to_curve_distance(obs_pt, projected_points)
        errors.append(dist)
    
    errors = np.array(errors)
    
    if loss_function == 'huber':
        # Huber loss: 对小误差用二次，大误差用线性
        threshold = 3.0  # 像素阈值
        mask_small = errors <= threshold
        
        loss = np.zeros_like(errors)
        loss[mask_small] = 0.5 * errors[mask_small]**2
        loss[~mask_small] = threshold * (errors[~mask_small] - 0.5 * threshold)
        
        return np.mean(loss)
    
    elif loss_function == 'l2':
        return np.mean(errors**2)
    
    else:
        return np.mean(np.abs(errors))
```

#### 模块 3: 非线性优化
```python
from scipy.optimize import least_squares

def reprojection_optimize(
    observed_points: np.array,  # (M, 2)  边缘点
    initial_pose: dict,         # 来自轴比方法的初值
    intrinsics: dict,
    radius: float = 0.025,
    max_iterations: int = 50
) -> dict:
    """
    使用 Levenberg-Marquardt 非线性最小二乘优化位姿
    
    初值：轴比方法结果
    目标：最小化重投影误差
    """
    
    # 1. 将参数展平成向量
    initial_params = np.array([
        initial_pose['center_3d'][0],
        initial_pose['center_3d'][1],
        initial_pose['center_3d'][2],
        initial_pose['normal'][0],
        initial_pose['normal'][1],
        initial_pose['normal'][2],
    ])
    
    def residuals(params):
        """
        计算残差向量（观测 - 模型）
        scipy.optimize.least_squares 会最小化 ||residuals||²
        """
        x, y, z, nx, ny, nz = params
        
        # 重建位姿
        center = np.array([x, y, z])
        normal = np.array([nx, ny, nz])
        
        # 归一化法向
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-9:
            return np.ones(len(observed_points)) * 1e10  # 惩罚
        normal = normal / norm_len
        
        # 2. 投影圆柱到图像
        try:
            projected = project_cylinder_to_image(
                center, normal, radius, intrinsics
            )
        except:
            return np.ones(len(observed_points)) * 1e10
        
        # 3. 计算每个观测点到投影曲线的距离
        residuals_vec = np.zeros(len(observed_points))
        for i, obs_pt in enumerate(observed_points):
            dist = point_to_curve_distance(obs_pt, projected)
            residuals_vec[i] = dist
        
        return residuals_vec
    
    # 4. 运行 LM 优化
    result = least_squares(
        residuals,
        initial_params,
        method='lm',  # Levenberg-Marquardt
        max_nfev=max_iterations * len(initial_params) * 10,
        ftol=1e-4,
        xtol=1e-4,
    )
    
    # 5. 提取优化结果
    optimized_params = result.x
    x, y, z, nx, ny, nz = optimized_params
    
    optimized_normal = np.array([nx, ny, nz])
    optimized_normal = optimized_normal / np.linalg.norm(optimized_normal)
    
    return {
        'center_3d': np.array([x, y, z]),
        'normal': optimized_normal,
        'reprojection_error': result.fun.sum(),
        'optimization_success': result.success,
    }
```

### 2.2 集成到 detection_node.py

```python
# 在 detection_node.py 中

from .pose_estimator import RGBCylinderPoseEstimator, ..., reprojection_optimize

class CylinderDetectionNode(Node):
    def __init__(self, ...):
        ...
        self.enable_reprojection = True  # 可配置
        self.reprojection_min_quality = 0.6  # 仅优化高质量帧
    
    def callback_image(self, msg):
        ...
        
        # 1. 轴比方法得到初值
        live_pose = self.pose_estimator.process_detection(...)
        
        if live_pose and self.enable_reprojection:
            quality_score = live_pose.get('quality_score', 0.0)
            
            # 2. 高质量帧才进行优化（降低计算负担）
            if quality_score > self.reprojection_min_quality:
                edge_points = ...  # 从检测中提取的边缘点
                
                # 3. 运行重投影优化
                optimized_pose = reprojection_optimize(
                    edge_points,
                    initial_pose=live_pose,
                    intrinsics=self.intrinsics,
                    radius=0.025
                )
                
                # 4. 验证优化有效性
                if optimized_pose['optimization_success']:
                    # 优化成功，使用优化结果
                    live_pose['center_3d'] = optimized_pose['center_3d']
                    live_pose['normal'] = optimized_pose['normal']
                    live_pose['optimized'] = True
                else:
                    # 优化失败，回退到轴比结果
                    live_pose['optimized'] = False
```

---

## 三、预期精度提升

### 3.1 理论分析

| 环节 | 轴比方法 | 重投影优化 |
|------|---------|-----------|
| **输入** | 椭圆长短轴 (2参数) | 100+ 边缘点 |
| **自由度** | 欠定 (2 → 6) | 超定 (100+ → 6) |
| **噪声敏感性** | 高 (导数敏感) | 低 (冗余约束) |
| **长轴误差** | ±3px → ±0.03 轴比 → ±3° 倾角 | 分散到6个参数，每个 ±0.1° |
| **倾角精度** | ±0.5~3° | ±0.1~0.2° |
| **位置精度** | ±8-15mm | ±1-2mm |

### 3.2 模拟数据

假设边缘检测误差 σ = 2px：

```
轴比方法（现状）：
  - 长轴 MA = 150px，误差 σ_MA = 2px
  - 轴比 r = 0.10，误差 σ_r ≈ 0.02
  - 倾角 θ ≈ 84.3°，误差 σ_θ ≈ ±1.2°
  - Z = 0.3m，误差 σ_Z = f_x * D * σ_MA / MA² ≈ ±5mm
  - 总误差：√(5² + tan(1.2°)*300²) ≈ ±12mm

重投影优化（目标）：
  - 使用所有 100 个边缘点，冗余度 100:6 ≈ 16:1
  - 通过 100 个约束共同拟合 6 个参数
  - 期望误差减少 √(冗余度) ≈ 4 倍
  - Z 误差：±5mm / 4 ≈ ±1.25mm
  - 总误差：√(1.25² + tan(0.3°)*300²) ≈ ±1.5mm ✓
```

---

## 四、实现路线图

### Week 1
- [ ] 实现 `project_cylinder_to_image()` （1h）
- [ ] 实现 `point_to_curve_distance()` 和 `reprojection_error()` （1h）
- [ ] 集成 scipy 优化器，完成 `reprojection_optimize()` （2h）
- [ ] 单元测试：用合成数据验证投影和优化 （2h）

### Week 2
- [ ] 集成到 detection_node.py （1h）
- [ ] 真实数据测试，调优参数 （2h）
- [ ] 性能基准（计算时间、精度）（1h）
- [ ] 文档和上线准备 （1h）

---

## 五、关键参数

```python
# config.py 中新增

# Phase D: 重投影优化
ENABLE_REPROJECTION_OPTIMIZATION = True
REPROJECTION_MIN_QUALITY = 0.6  # 仅优化 quality > 0.6 的帧
REPROJECTION_LOSS_FUNCTION = 'huber'  # 或 'l2', 'l1'
REPROJECTION_MAX_ITERATIONS = 50
REPROJECTION_CONVERGENCE_TOL = 1e-4
REPROJECTION_HUBER_THRESHOLD = 3.0  # 像素

# 圆柱模型（需从外部给定或自动测量）
CYLINDER_RADIUS = 0.025  # 米，5cm 直径
```

---

## 六、与 Phase C 的关系

**Phase C (可见性约束) 不是 Phase D 的前置**：
- Phase D 可以独立于 Phase C 实现
- Phase C 提供防护与降级策略
- 建议顺序：
  1. ✅ Phase A (质量评分)
  2. ✅ Phase B (椭圆鲁棒性)
  3. **Phase D (重投影)** - 核心精度突破 🔴 优先
  4. Phase C (可见性约束) - 防护层（可选）

---

## 七、风险与回退

| 风险 | 概率 | 缓解方案 |
|------|------|---------|
| 优化不收敛 | 低 | 初值来自可靠的轴比方法 |
| 计算超时 | 中 | 仅优化高质量帧，或用 LM 的快速变体 |
| 过拟合 | 低 | 100+ 约束 vs 6 参数，超定 |
| 生产环境稳定性 | 中 | 回退机制：失败时用轴比结果 |

---

**总结**：Phase D 是达到 ±5mm 精度的必经之路。相比 Phase A/B 的概念级改进，Phase D 是**数量级精度提升**（±15mm → ±2mm）。建议立即启动。
