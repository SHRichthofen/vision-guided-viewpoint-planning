# pose_estimator.py
import cv2
import numpy as np
import math
from . import config as cfg

class RGBCylinderPoseEstimator:
    def __init__(self, intr):
        self.fx = intr.fx
        self.fy = intr.fy
        self.cx = intr.ppx
        self.cy = intr.ppy
        # 初始化 CLAHE
        self.clahe = cv2.createCLAHE(clipLimit=cfg.CLAHE_CLIP, tileGridSize=cfg.CLAHE_GRID)

    def auto_canny(self, image, sigma=cfg.CANNY_SIGMA):
        v = np.median(image)
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        return cv2.Canny(image, lower, upper)

    def _ellipse_norm_distance(self, points_xy, ellipse):
        """计算点到椭圆的归一化半径距离。"""
        (xc, yc), (MA, ma), angle = ellipse
        a, b = MA / 2.0, ma / 2.0
        if a <= 1e-6 or b <= 1e-6:
            return np.full((len(points_xy),), 1e6, dtype=np.float32)

        rad = np.deg2rad(angle)
        cos_a, sin_a = np.cos(rad), np.sin(rad)
        dx = points_xy[:, 0] - xc
        dy = points_xy[:, 1] - yc
        x_rot = dx * cos_a + dy * sin_a
        y_rot = -dx * sin_a + dy * cos_a
        return np.sqrt((x_rot / a) ** 2 + (y_rot / b) ** 2)

    def _ellipse_perimeter(self, MA, ma):
        """Ramanujan 二阶周长近似。"""
        a = MA / 2.0
        b = ma / 2.0
        if a <= 1e-6 or b <= 1e-6:
            return 0.0
        h = ((a - b) ** 2) / ((a + b) ** 2 + 1e-6)
        return math.pi * (a + b) * (1 + (3 * h) / (10 + math.sqrt(4 - 3 * h) + 1e-6))

    def check_body_rim_consistency(self, body_bbox, ellipse_2d, normal):
        """Body-Rim 联合约束一致性分数（0~1）。"""
        if body_bbox is None or ellipse_2d is None or normal is None:
            return 1.0

        try:
            _, _, bw, bh = body_bbox
            body_dir = np.array([0.0, 1.0]) if bh >= bw else np.array([1.0, 0.0])

            proj = np.array([normal[0] * self.fx, normal[1] * self.fy], dtype=np.float32)
            proj_norm = np.linalg.norm(proj)
            if proj_norm < 1e-6:
                return 0.5
            proj /= proj_norm

            consistency = abs(float(np.dot(proj, body_dir)))
            return float(np.clip(consistency, 0.0, 1.0))
        except Exception:
            return 0.5

    def compute_quality_score(self, pose_dict, ellipse_metrics):
        """计算质量分数与子项。"""
        if pose_dict is None or ellipse_metrics is None:
            return 0.0, {
                'q_area': 0.0,
                'q_coverage': 0.0,
                'q_residual': 0.0,
                'q_axis': 0.0,
                'consistency_score': 0.0,
            }

        MA = pose_dict['ellipse_2d'][1][0]
        ma = pose_dict['ellipse_2d'][1][1]
        area = math.pi * (MA / 2.0) * (ma / 2.0)

        # q_area: 椭圆面积有效区间
        if area < cfg.ELLIPSE_AREA_MIN_PX2:
            q_area = float(np.clip(area / (cfg.ELLIPSE_AREA_MIN_PX2 + 1e-6), 0.0, 1.0))
        elif area > cfg.ELLIPSE_AREA_MAX_PX2:
            q_area = float(np.clip((2.0 * cfg.ELLIPSE_AREA_MAX_PX2 - area) / (cfg.ELLIPSE_AREA_MAX_PX2 + 1e-6), 0.0, 1.0))
        else:
            q_area = 1.0

        # q_coverage: 边缘覆盖率
        coverage = float(ellipse_metrics.get('coverage', 0.0))
        q_coverage = float(np.clip(coverage / (cfg.RIM_COVERAGE_MIN + 1e-6), 0.0, 1.0))

        # q_residual: 残差
        residual = float(ellipse_metrics.get('residual', cfg.ELLIPSE_RESIDUAL_MAX * 2.0))
        q_residual = float(np.clip(1.0 - residual / (cfg.ELLIPSE_RESIDUAL_MAX + 1e-6), 0.0, 1.0))

        # q_axis: 轴比合理性
        axis_ratio = float(pose_dict.get('axis_ratio', 0.0))
        if cfg.AXIS_RATIO_QUALITY_MIN <= axis_ratio <= cfg.AXIS_RATIO_QUALITY_MAX:
            q_axis = 1.0
        elif axis_ratio < cfg.AXIS_RATIO_QUALITY_MIN:
            q_axis = float(np.clip(axis_ratio / (cfg.AXIS_RATIO_QUALITY_MIN + 1e-6), 0.0, 1.0))
        else:
            span = max(1e-6, 1.0 - cfg.AXIS_RATIO_QUALITY_MAX)
            q_axis = float(np.clip((1.0 - axis_ratio) / span, 0.0, 1.0))

        q = (
            0.20 * q_area
            + 0.25 * q_coverage
            + 0.35 * q_residual
            + 0.20 * q_axis
        )

        consistency_score = float(np.clip(ellipse_metrics.get('consistency_score', 1.0), 0.0, 1.0))
        q *= (0.7 + 0.3 * consistency_score)
        q = float(np.clip(q, 0.0, 1.0))

        return q, {
            'q_area': q_area,
            'q_coverage': q_coverage,
            'q_residual': q_residual,
            'q_axis': q_axis,
            'consistency_score': consistency_score,
            'coverage': coverage,
            'residual_px': residual,
            'ellipse_area_px2': area,
        }

    def refine_ellipse_from_hull(self, all_points, hull_ellipse):
        """基于凸包粗拟合的鲁棒精修。"""
        (xc, yc), (MA, ma), angle = hull_ellipse
        
        rad_angle = np.deg2rad(angle)
        cos_a, sin_a = np.cos(rad_angle), np.sin(rad_angle)
        a, b = MA / 2, ma / 2
        
        if a <= 0 or b <= 0: return hull_ellipse

        # 变换到椭圆坐标系
        pts = all_points.reshape(-1, 2).astype(np.float32)
        dx = pts[:, 0] - xc
        dy = pts[:, 1] - yc
        
        x_rot = dx * cos_a + dy * sin_a
        y_rot = -dx * sin_a + dy * cos_a
        
        norm_dist = np.sqrt((x_rot / a)**2 + (y_rot / b)**2)
        
        # 只保留贴近椭圆线的点 (Inliers)
        inliers_mask = (norm_dist > 0.80) & (norm_dist < 1.20)
        inliers = pts[inliers_mask]

        # 近距不完整时放宽支持部分弧段
        if len(inliers) < 20:
            inliers_mask = (norm_dist > 0.70) & (norm_dist < 1.30)
            inliers = pts[inliers_mask]
        
        if len(inliers) < 20: 
            return hull_ellipse

        # 近似加权最小二乘：按“靠近边缘”重复采样
        closeness = 1.0 / (np.abs(norm_dist[inliers_mask] - 1.0) + 0.05)
        reps = np.clip(np.round(closeness), 1, 6).astype(np.int32)
        weighted_points = np.repeat(inliers, reps, axis=0)
        if len(weighted_points) < 20:
            weighted_points = inliers
            
        try:
            refined_ellipse = cv2.fitEllipse(weighted_points.astype(np.int32))
            (rx, ry), (rd1, rd2), rangle = refined_ellipse
            rMA, rma = (rd1, rd2) if rd1 >= rd2 else (rd2, rd1)
            real_angle = rangle if rd1 >= rd2 else rangle + 90

            return ((rx, ry), (rMA, rma), real_angle % 180.0)
        except:
            return hull_ellipse

    def process_detection(
        self,
        image,
        precise_rim_mask,
        crop_offset,
        body_bbox=None,
        depth_prior_m=None,
        depth_prior_confidence=0.0
    ):
        """ 主处理流程: Crop -> CLAHE -> Canny -> Hull -> Fit -> Refine """
        crop_x1, crop_y1 = crop_offset
        
        # 1. 动态截取 ROI (只处理 mask 覆盖的区域，加速)
        ys, xs = np.where(precise_rim_mask > 0)
        if len(ys) == 0: return None, "Empty Mask", [], None, (0,0)
        
        min_x, max_x = np.min(xs), np.max(xs)
        min_y, max_y = np.min(ys), np.max(ys)
        pad = 20
        x1 = max(0, min_x - pad); y1 = max(0, min_y - pad)
        x2 = min(image.shape[1], max_x + pad); y2 = min(image.shape[0], max_y + pad)
        
        roi_img = image[y1:y2, x1:x2]
        roi_mask = precise_rim_mask[y1:y2, x1:x2]
        
        if roi_img.size == 0: return None, "Empty ROI", [], None, (x1, y1)

        # 2. 图像增强与边缘提取
        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.bilateralFilter(gray, 5, 50, 50)
        enhanced = self.clahe.apply(gray_blur)
        
        edges = self.auto_canny(enhanced)
        edges_filtered = cv2.bitwise_and(edges, edges, mask=roi_mask)
        
        # 3. 收集边缘点云
        contours, _ = cv2.findContours(edges_filtered, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        all_edge_points = []
        debug_vis_img = cv2.cvtColor(edges_filtered, cv2.COLOR_GRAY2BGR)

        for cnt in contours:
            if cv2.arcLength(cnt, False) < cfg.MIN_EDGE_LENGTH: continue
            all_edge_points.append(cnt)

        if not all_edge_points:
            return None, "No Valid Edges", [], debug_vis_img, (x1, y1)
        
        # 4. 凸包粗拟合
        combined_contour = np.vstack(all_edge_points)
        hull = cv2.convexHull(combined_contour)
        if len(hull) < 5: return None, "Hull too small", [], debug_vis_img, (x1, y1)
        
        try:
            coarse_ellipse = cv2.fitEllipse(hull)
        except:
            return None, "Fit Failed", [], debug_vis_img, (x1, y1)

        # 5. 精修拟合
        final_ellipse = self.refine_ellipse_from_hull(combined_contour, coarse_ellipse)
        
        # 画出最终结果用于调试
        cv2.ellipse(debug_vis_img, final_ellipse, (0, 0, 255), 2)

        # 6. 计算最终参数与位姿
        (e_x, e_y), (d1, d2), angle = final_ellipse
        MA, ma = (d1, d2) if d1 >= d2 else (d2, d1)
        real_angle = angle if d1 >= d2 else angle + 90
        
        if MA <= 0 or ma <= 0: return None, "Invalid Ellipse", [], debug_vis_img, (x1, y1)
        if ma / MA < cfg.MIN_AXIS_RATIO: return None, "Too Flat", [], debug_vis_img, (x1, y1)

        # 转换回全图坐标
        global_cx = e_x + x1
        global_cy = e_y + y1
        full_img_ellipse = ((global_cx, global_cy), (MA, ma), real_angle)

        # 3D 解算（椭圆几何）
        Z_ellipse = self.fx * cfg.REAL_TUBE_DIAMETER / MA
        Z = float(Z_ellipse)

        # 深度先验融合（可选）
        depth_conf = float(np.clip(depth_prior_confidence, 0.0, 1.0))
        z_fusion_weight = 0.0
        near_circle = (ma / MA) >= cfg.NEAR_CIRCLE_AXIS_RATIO
        if depth_prior_m is not None and depth_prior_m > 1e-6 and depth_conf > 1e-3:
            z_fusion_weight = float(np.clip(depth_conf * cfg.DEPTH_PRIOR_WEIGHT_MAX, cfg.DEPTH_PRIOR_WEIGHT_MIN, cfg.DEPTH_PRIOR_WEIGHT_MAX))
            if near_circle:
                z_fusion_weight = max(z_fusion_weight, cfg.DEPTH_PRIOR_WEIGHT_NEAR_CIRCLE)

            z_delta = abs(float(depth_prior_m) - float(Z_ellipse))
            if z_delta > cfg.DEPTH_PRIOR_MAX_DELTA_M:
                z_fusion_weight *= 0.25

            Z = float(z_fusion_weight * float(depth_prior_m) + (1.0 - z_fusion_weight) * float(Z_ellipse))

        X = (global_cx - self.cx) * Z / self.fx
        Y = (global_cy - self.cy) * Z / self.fy

        ratio_clamped = min(1.0, max(-1.0, ma / MA))
        tilt = math.acos(ratio_clamped)
        phi = math.radians(real_angle + 90)
        nx = math.sin(tilt) * math.cos(phi)
        ny = math.sin(tilt) * math.sin(phi)
        nz = -math.cos(tilt) 

        normal = np.array([nx, ny, nz], dtype=np.float32)
        center_3d = np.array([X, Y, Z], dtype=np.float32)

        # 法向半球一致性：解决“向内/向外”符号翻转
        # 以相机光心为原点，center_3d 指向目标中心。
        # NORMAL_POINT_AWAY_FROM_CAMERA=True 时，法向与 center_3d 同向（背离相机）。
        dot_view = float(np.dot(normal, center_3d))
        if cfg.NORMAL_POINT_AWAY_FROM_CAMERA:
            if dot_view < 0.0:
                normal = -normal
        else:
            if dot_view > 0.0:
                normal = -normal

        normal /= (np.linalg.norm(normal) + 1e-6)

        pose_result = {
            "center_3d": center_3d,
            "normal": normal,
            "ellipse_2d": full_img_ellipse,
            "axis_ratio": ma / MA,
            "z_ellipse": float(Z_ellipse),
            "z_prior": None if depth_prior_m is None else float(depth_prior_m),
            "z_fused": float(Z),
            "z_fusion_weight": float(z_fusion_weight),
            "depth_prior_confidence": float(depth_conf),
            "near_circle_degenerate": bool(near_circle)
        }

        # Phase A/B: 质量指标计算
        local_pts = combined_contour.reshape(-1, 2).astype(np.float32)
        norm_dist_local = self._ellipse_norm_distance(local_pts, final_ellipse)
        avg_radius = (MA + ma) / 4.0
        residual_px = float(np.mean(np.abs(norm_dist_local - 1.0)) * avg_radius)
        perimeter = self._ellipse_perimeter(MA, ma)
        coverage = float(min(1.0, len(local_pts) / (perimeter + 1e-6)))
        consistency_score = self.check_body_rim_consistency(body_bbox, full_img_ellipse, pose_result['normal'])

        fit_residual_score = float(np.clip(1.0 - residual_px / (cfg.ELLIPSE_RESIDUAL_MAX + 1e-6), 0.0, 1.0))
        ellipse_fit_score = float(np.clip(0.6 * fit_residual_score + 0.4 * coverage, 0.0, 1.0))

        metrics = {
            'coverage': coverage,
            'residual': residual_px,
            'consistency_score': consistency_score,
            'ellipse_fit_score': ellipse_fit_score,
        }
        quality_score, quality_components = self.compute_quality_score(pose_result, metrics)

        pose_result['quality_score'] = quality_score
        pose_result['quality_components'] = quality_components
        pose_result['consistency_score'] = consistency_score
        pose_result['ellipse_fit_score'] = ellipse_fit_score
        
        return pose_result, "Success", [], debug_vis_img, (x1, y1)
