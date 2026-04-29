# 快速修复清单：Phase A/B + Phase D 并行推进方案

**总体目标**：当前 ±10-15mm → 目标 ±5mm  
**时间表**：
- Phase A 改进：1 天（立即）
- Phase D 开发：2-3 天（并行）
- 集成验证：1 天

---

## 第一步：Phase A 质量评分重设计（今天，4h）

### 任务 1.1: 修改 `compute_quality_score()` 函数

**位置**: `pose_estimator.py` L8-67

**改动**：从轴比单维评分 → 多维综合评分

```python
def compute_quality_score_v2(pose_dict, edge_metrics=None, image_shape=None):
    """
    重新设计的多维质量评分（不依赖轴比）
    
    评分维度：
    1. 覆盖率 (coverage): 边缘点覆盖椭圆周长的比例
    2. 残差 (residual): 拟合误差（点到椭圆线距离）
    3. 清晰度 (clarity): Canny 梯度强度
    4. 可见性 (visibility): 椭圆中心离边界的距离
    
    权重：
    - coverage: 30% (最重要，反映了观测完整性)
    - residual: 25% (拟合质量)
    - clarity: 25%  (边缘清晰度)
    - visibility: 20% (是否被裁剪)
    """
    
    # 初始化
    quality_components = {}
    
    # === 维度1: 覆盖率 ===
    if edge_metrics and 'coverage' in edge_metrics:
        coverage = edge_metrics['coverage']
        # 覆盖率 50% → 0.5 分, 100% → 1.0 分
        q_coverage = min(1.0, max(0.0, coverage * 2))
    else:
        q_coverage = 0.5  # 缺失时打默认值
    
    quality_components['q_coverage'] = q_coverage
    
    # === 维度2: 残差 ===
    if edge_metrics and 'residual' in edge_metrics:
        residual = edge_metrics['residual']
        # 残差 0px → 1.0 分, 3px → 0.3 分, 5px+ → 0.0 分
        q_residual = max(0.0, 1.0 - residual / 3.0 * 0.7)
    else:
        q_residual = 0.5
    
    quality_components['q_residual'] = q_residual
    
    # === 维度3: 清晰度（梯度强度）===
    if edge_metrics and 'mean_gradient' in edge_metrics:
        mean_grad = edge_metrics['mean_gradient']
        # 梯度强度 0-255，期望 > 100 为清晰
        q_clarity = min(1.0, max(0.0, mean_grad / 150.0))
    else:
        q_clarity = 0.5
    
    quality_components['q_clarity'] = q_clarity
    
    # === 维度4: 可见性（椭圆不被裁剪）===
    if pose_dict and 'ellipse_2d' in pose_dict and image_shape:
        (e_x, e_y), (MA, ma), angle = pose_dict['ellipse_2d']
        img_h, img_w = image_shape
        
        # 椭圆边界到图像边界的距离
        margins = [
            e_x - MA/2,        # 左边界
            img_w - (e_x + MA/2),  # 右边界
            e_y - ma/2,        # 上边界
            img_h - (e_y + ma/2)   # 下边界
        ]
        
        min_margin = min(margins) if margins else 0
        
        if min_margin < 0:
            # 被裁剪，严重扣分
            q_visibility = 0.3
        elif min_margin < 20:
            # 靠近边界，轻度扣分
            q_visibility = 0.7
        else:
            # 充分可见
            q_visibility = 1.0
    else:
        q_visibility = 0.5
    
    quality_components['q_visibility'] = q_visibility
    
    # === 综合评分（加权组合）===
    quality_score = (
        0.30 * q_coverage +
        0.25 * q_residual +
        0.25 * q_clarity +
        0.20 * q_visibility
    )
    
    quality_score = float(max(0.0, min(1.0, quality_score)))
    
    return {
        'quality_score': quality_score,
        'q_coverage': float(q_coverage),
        'q_residual': float(q_residual),
        'q_clarity': float(q_clarity),
        'q_visibility': float(q_visibility),
        'components': quality_components
    }
```

### 任务 1.2: 在 `process_detection()` 中计算梯度强度

**位置**: `pose_estimator.py` L600 之前

```python
# === 计算梯度强度（用于清晰度评分）===
# 在 process_detection() 末尾，在调用 compute_quality_score 前

gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)

# Sobel 梯度计算
grad_x = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
grad_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
grad_magnitude = np.sqrt(grad_x**2 + grad_y**2)

# 仅在 mask 区域内计算平均梯度
mask_pixels = precise_rim_mask[y1:y2, x1:x2]
if np.any(mask_pixels > 0):
    mean_gradient = np.mean(grad_magnitude[mask_pixels > 0])
else:
    mean_gradient = 0.0

quality_metrics['mean_gradient'] = mean_gradient

# 然后调用新的评分函数
quality_result = compute_quality_score_v2(
    pose_result, 
    quality_metrics,
    image_shape=image.shape[:2]  # (height, width)
)
```

### 任务 1.3: 更新配置文件

**位置**: `config.py` L32-39

```python
# 原配置（注释掉）
# QUALITY_MIN_PUBLISH = 0.2             # 极端宽松
# QUALITY_MIN_FULL_POSE = 0.3

# 新配置（Phase A 改进版）
QUALITY_MIN_PUBLISH = 0.45              # 提升门控（之前0.2）
QUALITY_MIN_FULL_POSE = 0.60            # 完整姿态需要 0.6+

# 多维质量指标的阈值
QUALITY_COVERAGE_MIN = 0.50             # 覆盖率 > 50%
QUALITY_RESIDUAL_MAX = 3.0              # 残差 < 3px
QUALITY_CLARITY_MIN = 80.0              # 梯度强度 > 80
QUALITY_VISIBILITY_MIN = 0.50           # 可见性评分 > 0.5
```

---

## 第二步：融合参数调优（1h）

**位置**: `config.py` L43-50

```python
# 从旧参数
# BURST_SAMPLE_COUNT = 7           # 700ms 延迟

# 改为新参数
BURST_SAMPLE_COUNT = 3               # 300ms 延迟（更响应）
BURST_FUSION_ENABLED = True

# 新增：融合时过滤低质量样本
BURST_FUSION_MIN_QUALITY = 0.55      # 融合时只用质量 > 0.55 的样本
```

**修改 detection_node.py** 中的融合逻辑（可选，但推荐）：

```python
# 在 L251 处，改为有条件融合

if cfg.BURST_FUSION_ENABLED and len(self.burst_sample_buffers[track_id]) >= cfg.BURST_SAMPLE_COUNT:
    # 过滤出高质量样本
    high_quality_poses = [
        p for p in self.burst_sample_buffers[track_id]
        if p.get('quality_score', 0.0) >= cfg.BURST_FUSION_MIN_QUALITY
    ]
    
    if len(high_quality_poses) >= 3:  # 至少要有 3 个高质量样本
        fused_pose = fuse_burst_samples(high_quality_poses)
    else:
        # 样本不足，用单帧最高质量的
        fused_pose = max(self.burst_sample_buffers[track_id], 
                        key=lambda p: p.get('quality_score', 0.0))
```

---

## 第三步：自适应 Dilation（30min）

**位置**: `detection_node.py` - `run_rim_pipeline()` 方法

在调用 `cv2.dilate()` 前，改为动态计算：

```python
# 原代码（固定值）
# kernel = np.ones((RIM_MASK_DILATION, RIM_MASK_DILATION), np.uint8)

# 新代码（自适应）
body_w = target_tube['bbox_xywh'][2]  # body bbox 的宽度
adaptive_dilation = int(max(5, min(25, body_w * 0.15)))  # 宽度的 15%，范围 [5, 25]
kernel = np.ones((adaptive_dilation, adaptive_dilation), np.uint8)
```

---

## 第四步：Phase D 核心实现（Day 2-3）

### 任务 4.1: 在 pose_estimator.py 中添加重投影优化

**新增文件**: 在 `pose_estimator.py` 末尾添加以下模块

```python
# ============ Phase D: 重投影优化 ============

def project_cylinder_to_image(
    cylinder_center: np.ndarray,
    cylinder_normal: np.ndarray,
    radius: float,
    intrinsics: dict,
    sample_count: int = 100
) -> np.ndarray:
    """
    将3D圆柱投影到2D图像平面
    
    Returns:
        projected_points: (sample_count, 2)  图像坐标
    """
    
    # 归一化法向
    axis = cylinder_normal / (np.linalg.norm(cylinder_normal) + 1e-9)
    
    # 构建垂直于轴的两个正交向量
    if abs(axis[2]) < 0.9:
        perpendicular = np.array([0, 0, 1]) - axis[2] * axis
    else:
        perpendicular = np.array([1, 0, 0]) - axis[0] * axis
    
    perpendicular = perpendicular / (np.linalg.norm(perpendicular) + 1e-9)
    radial_1 = perpendicular
    radial_2 = np.cross(axis, radial_1)
    radial_2 = radial_2 / (np.linalg.norm(radial_2) + 1e-9)
    
    # 圆周采样
    angles = np.linspace(0, 2*np.pi, sample_count, endpoint=False)
    
    # 在3D空间中生成圆周上的点
    local_points = radius * (
        np.outer(np.cos(angles), radial_1) + 
        np.outer(np.sin(angles), radial_2)
    )  # (sample_count, 3)
    
    world_points = cylinder_center + local_points  # (sample_count, 3)
    
    # 投影到图像
    z = world_points[:, 2]
    x = world_points[:, 0]
    y = world_points[:, 1]
    
    # 避免 z <= 0
    z = np.maximum(z, 0.01)
    
    u = intrinsics['f_x'] * (x / z) + intrinsics['c_x']
    v = intrinsics['f_y'] * (y / z) + intrinsics['c_y']
    
    return np.column_stack([u, v])


def point_to_curve_distance(
    point: np.ndarray,
    curve_points: np.ndarray
) -> float:
    """
    计算点到曲线的最短距离
    """
    distances = np.linalg.norm(curve_points - point, axis=1)
    return np.min(distances)


def reprojection_optimize(
    observed_points: np.ndarray,
    initial_pose: dict,
    intrinsics: dict,
    radius: float = 0.025,
    max_iterations: int = 50
) -> dict:
    """
    使用 LM 算法优化位姿
    
    Args:
        observed_points: (M, 2) 边缘检测的点
        initial_pose: {'center_3d', 'normal'} 初值
        intrinsics: 相机内参
    
    Returns:
        optimized_pose: 优化后的位姿
    """
    
    from scipy.optimize import least_squares
    
    # 初值
    x0, y0, z0 = initial_pose['center_3d']
    nx0, ny0, nz0 = initial_pose['normal']
    
    initial_params = np.array([x0, y0, z0, nx0, ny0, nz0])
    
    def residuals(params):
        """
        计算残差向量
        """
        x, y, z, nx, ny, nz = params
        
        center = np.array([x, y, z])
        normal = np.array([nx, ny, nz])
        
        # 归一化法向
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-9:
            return np.ones(len(observed_points)) * 100.0
        
        normal = normal / norm_len
        
        # 投影圆柱到图像
        try:
            projected = project_cylinder_to_image(
                center, normal, radius, intrinsics, sample_count=120
            )
        except Exception:
            return np.ones(len(observed_points)) * 100.0
        
        # 计算每个观测点到投影曲线的距离
        residuals_vec = np.array([
            point_to_curve_distance(obs_pt, projected)
            for obs_pt in observed_points
        ])
        
        return residuals_vec
    
    # 运行优化
    try:
        result = least_squares(
            residuals,
            initial_params,
            method='lm',
            max_nfev=max_iterations * 6 * 50,
            ftol=1e-4,
            xtol=1e-4,
        )
        
        optimized_x, optimized_y, optimized_z, opt_nx, opt_ny, opt_nz = result.x
        
        optimized_normal = np.array([opt_nx, opt_ny, opt_nz])
        optimized_normal = optimized_normal / (np.linalg.norm(optimized_normal) + 1e-9)
        
        return {
            'center_3d': np.array([optimized_x, optimized_y, optimized_z]),
            'normal': optimized_normal,
            'optimization_success': result.success,
            'reprojection_error': np.sum(result.fun**2),
        }
    
    except Exception as e:
        # 优化失败，返回初值和失败标记
        return {
            'center_3d': initial_pose['center_3d'],
            'normal': initial_pose['normal'],
            'optimization_success': False,
            'reprojection_error': np.inf,
        }
```

### 任务 4.2: 在 detection_node.py 中集成优化器

**修改位置**: `detection_node.py` 的 `callback_image()` 方法，在发布前调用优化

```python
# 在 L250 的 publish_selected_target() 前添加

if live_pose and cfg.ENABLE_REPROJECTION_OPTIMIZATION:
    quality_score = live_pose.get('quality_score', 0.0)
    
    # 仅对高质量帧进行重投影优化
    if quality_score >= cfg.REPROJECTION_MIN_QUALITY:
        try:
            # 获取边缘点（从 all_edge_points 或重新提取）
            # 这里假设在 process_detection 中有 edge_points 返回
            
            optimized_pose = reprojection_optimize(
                observed_points=edge_points,
                initial_pose=live_pose,
                intrinsics=self.pose_estimator.intrinsics,
                radius=cfg.CYLINDER_RADIUS
            )
            
            if optimized_pose['optimization_success']:
                # 优化成功，更新位姿
                live_pose['center_3d'] = optimized_pose['center_3d']
                live_pose['normal'] = optimized_pose['normal']
                live_pose['optimized'] = True
            else:
                live_pose['optimized'] = False
        
        except Exception as e:
            self.get_logger().warn(f"重投影优化失败: {e}")
            live_pose['optimized'] = False
```

### 任务 4.3: 配置参数

**位置**: `config.py` 末尾

```python
# ================= Phase D: 重投影优化 =================
ENABLE_REPROJECTION_OPTIMIZATION = True
REPROJECTION_MIN_QUALITY = 0.60         # 仅优化 quality > 0.6 的帧
REPROJECTION_MAX_ITERATIONS = 50
CYLINDER_RADIUS = 0.025                 # 米，5cm 直径

REPROJECTION_LOSS_FUNCTION = 'l2'       # L2 loss（平方误差）
```

---

## 验证清单

### 编译检查
```bash
cd /home/arnoyin/grad_proj/qrc_hand/arm_ws
colcon build --packages-select vision_detection
```

### 运行检查
```bash
source install/setup.bash
ros2 launch vision_detection detection.launch.py
# 观察日志：quality_score 应该分布更合理（不全是 > 0.5）
# 观察是否有 "优化成功" 的日志
```

### 数据验证
1. 记录 100 帧数据，统计 quality_score 分布
   - Phase A 改进版：应该有 30-40% 的帧被拒发（质量 < 0.45）
2. 对比 Phase D 优化前后的位置标准差
   - 期望：σ 从 8mm 降到 2-3mm

---

## 时间表总结

| 任务 | 工作量 | 优先级 | 预计完成 |
|------|--------|--------|-----------|
| 1.1 重设计质量评分 | 2h | 🔴 立即 | T+2h |
| 1.2 梯度计算 | 1h | 🔴 立即 | T+3h |
| 1.3 参数更新 | 0.5h | 🔴 立即 | T+3.5h |
| 2 融合参数 | 1h | 🔴 立即 | T+4.5h |
| 3 自适应 dilation | 0.5h | 🔴 立即 | T+5h |
| 4.1 重投影模块 | 4h | 🟡 Day 2-3 | T+9h |
| 4.2 集成到节点 | 2h | 🟡 Day 2-3 | T+11h |
| 4.3 配置和测试 | 1h | 🟡 Day 2-3 | T+12h |
| **总计** | **12h** | - | **1.5 天** |

---

## 风险与回退

| 风险 | 概率 | 缓解 |
|------|------|------|
| Phase A 改进后质量分仍不理想 | 中 | 立即启动 Phase D |
| Phase D 优化超时（>50ms） | 低 | 仅对 5 帧/秒优化，其他帧跳过 |
| 生产环境不稳定 | 低 | 添加 try-except，失败时回退轴比结果 |
| 相机内参不准 | 中 | 导致投影偏离，重新标定 |

---

**建议**: 立即执行第一步（Phase A 改进），同时启动第四步（Phase D 开发），力争 1.5 天内完成 ±5mm 精度目标。
