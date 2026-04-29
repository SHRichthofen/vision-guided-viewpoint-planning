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
        self.clahe = cv2.createCLAHE(
            clipLimit=cfg.CLAHE_CLIP,
            tileGridSize=cfg.CLAHE_GRID
        )
        self._fit_ellipse_fn = getattr(cv2, "fitEllipseAMS", cv2.fitEllipse)

    def auto_canny(self, image, sigma=cfg.CANNY_SIGMA):
        v = np.median(image)
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        return cv2.Canny(image, lower, upper)

    def _normalize_ellipse(self, ellipse):
        (xc, yc), (d1, d2), angle = ellipse
        if d1 >= d2:
            major, minor = d1, d2
            real_angle = angle
        else:
            major, minor = d2, d1
            real_angle = angle + 90.0
        return (
            (float(xc), float(yc)),
            (float(major), float(minor)),
            float(real_angle % 180.0),
        )

    def _fit_ellipse_from_points(self, points_xy):
        pts = np.asarray(points_xy, dtype=np.float32).reshape(-1, 2)
        if len(pts) < 5:
            return None

        try:
            ellipse = self._fit_ellipse_fn(pts)
        except Exception:
            try:
                ellipse = cv2.fitEllipse(pts)
            except Exception:
                return None

        ellipse = self._normalize_ellipse(ellipse)
        if ellipse[1][0] <= 1e-6 or ellipse[1][1] <= 1e-6:
            return None
        return ellipse

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

    def _ellipse_param_angles(self, points_xy, ellipse):
        pts = np.asarray(points_xy, dtype=np.float32).reshape(-1, 2)
        (xc, yc), (MA, ma), angle = ellipse
        a = max(MA / 2.0, 1e-6)
        b = max(ma / 2.0, 1e-6)
        rad = np.deg2rad(angle)
        cos_a, sin_a = np.cos(rad), np.sin(rad)
        dx = pts[:, 0] - xc
        dy = pts[:, 1] - yc
        x_rot = dx * cos_a + dy * sin_a
        y_rot = -dx * sin_a + dy * cos_a
        return np.arctan2(y_rot / b, x_rot / a)

    def _estimate_angular_coverage(self, points_xy, ellipse, num_bins=72):
        pts = np.asarray(points_xy, dtype=np.float32).reshape(-1, 2)
        if len(pts) == 0:
            return 0.0
        angles = self._ellipse_param_angles(pts, ellipse)
        bins = np.floor((angles + math.pi) / (2.0 * math.pi) * num_bins).astype(np.int32)
        bins = np.clip(bins, 0, num_bins - 1)
        return float(len(np.unique(bins)) / float(num_bins))

    def _fit_line_residual(self, points_xy):
        pts = np.asarray(points_xy, dtype=np.float32).reshape(-1, 2)
        if len(pts) < 2:
            return 1e6, np.array([1.0, 0.0], dtype=np.float32)

        vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
        vx = float(vx)
        vy = float(vy)
        x0 = float(x0)
        y0 = float(y0)
        normal = np.array([-vy, vx], dtype=np.float32)
        distances = np.abs((pts[:, 0] - x0) * normal[0] + (pts[:, 1] - y0) * normal[1])
        direction = np.array([vx, vy], dtype=np.float32)
        direction /= (np.linalg.norm(direction) + 1e-6)
        return float(np.mean(distances)), direction

    def _ellipse_overlap_metrics(self, ellipse, rim_mask, boundary_band):
        fill_mask = np.zeros_like(rim_mask, dtype=np.uint8)
        ring_mask = np.zeros_like(rim_mask, dtype=np.uint8)
        band_half_width = max(1, int(round(cfg.RIM_BOUNDARY_BAND_MAX / 2.0)))
        cv2.ellipse(fill_mask, ellipse, 255, -1)
        cv2.ellipse(ring_mask, ellipse, 255, band_half_width)

        fill_intersection = float(np.count_nonzero(cv2.bitwise_and(fill_mask, rim_mask)))
        fill_union = float(np.count_nonzero(cv2.bitwise_or(fill_mask, rim_mask)))
        fill_iou = fill_intersection / (fill_union + 1e-6)

        ring_overlap = float(np.count_nonzero(cv2.bitwise_and(ring_mask, boundary_band)))
        ring_norm = float(np.count_nonzero(ring_mask))
        ring_ratio = ring_overlap / (ring_norm + 1e-6)

        mask_overlap = 0.55 * ring_ratio + 0.45 * fill_iou
        return float(mask_overlap), float(ring_ratio), float(fill_iou)

    def _mask_centroid_score(self, ellipse, rim_mask):
        ys, xs = np.where(rim_mask > 0)
        if len(xs) == 0:
            return 0.0
        centroid = np.array([np.mean(xs), np.mean(ys)], dtype=np.float32)
        center = np.array(ellipse[0], dtype=np.float32)
        diag = max(1.0, float(np.linalg.norm(np.array(rim_mask.shape[::-1], dtype=np.float32))))
        offset = float(np.linalg.norm(center - centroid))
        return float(np.clip(1.0 - offset / (0.35 * diag + 1e-6), 0.0, 1.0))

    def _summarize_candidates(self, candidates):
        return [
            {
                'source': c['source'],
                'score': float(c['score']),
                'support_ratio': float(c['support_ratio']),
                'coverage': float(c['coverage']),
                'mask_overlap': float(c['mask_overlap']),
                'residual_px': float(c['residual_px']),
                'segment_count': int(c['segment_count']),
            }
            for c in candidates[:cfg.DEBUG_TOP_CANDIDATES]
        ]

    def extract_rim_boundary_band(self, roi_mask):
        """从 rim mask 生成一个窄边界带，避免整块 mask 直接主导拟合。"""
        mask = (roi_mask > 0).astype(np.uint8) * 255
        if np.count_nonzero(mask) < 20:
            return None, None, [], 0

        area = float(np.count_nonzero(mask))
        eq_radius = max(2.0, math.sqrt(area / math.pi))
        close_size = int(round(eq_radius * cfg.RIM_MASK_CLOSE_SCALE))
        close_size = int(np.clip(close_size, cfg.RIM_MASK_CLOSE_MIN, cfg.RIM_MASK_CLOSE_MAX))
        if close_size % 2 == 0:
            close_size += 1

        band_half_width = int(round(eq_radius * cfg.RIM_BOUNDARY_BAND_RADIUS_RATIO))
        band_half_width = int(np.clip(
            band_half_width,
            cfg.RIM_BOUNDARY_BAND_MIN,
            cfg.RIM_BOUNDARY_BAND_MAX
        ))

        close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_size, close_size))
        band_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (2 * band_half_width + 1, 2 * band_half_width + 1)
        )

        cleaned = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        dilated = cv2.dilate(cleaned, band_kernel, iterations=1)
        eroded = cv2.erode(cleaned, band_kernel, iterations=1)
        boundary_band = cv2.subtract(dilated, eroded)

        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        return cleaned, boundary_band, contours, band_half_width

    def detect_arc_support_segments(self, edges_filtered, boundary_band):
        """任务化裁剪版 arc-support segments：先从 band 内边缘中提取弧段与短线段。"""
        contours, _ = cv2.findContours(edges_filtered, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        arc_segments = []

        for idx, cnt in enumerate(contours):
            pts = cnt.reshape(-1, 2).astype(np.float32)
            if len(pts) < cfg.ARC_SEGMENT_MIN_POINTS:
                continue

            arc_len = float(cv2.arcLength(cnt, False))
            if arc_len < cfg.MIN_EDGE_LENGTH:
                continue

            ellipse_hint = self._fit_ellipse_from_points(pts)
            if ellipse_hint is None:
                continue

            norm_dist = self._ellipse_norm_distance(pts, ellipse_hint)
            avg_radius = (ellipse_hint[1][0] + ellipse_hint[1][1]) / 4.0
            residual_px = float(np.mean(np.abs(norm_dist - 1.0)) * avg_radius)
            coverage = self._estimate_angular_coverage(pts, ellipse_hint)
            span_deg = float(coverage * 360.0)

            if residual_px > cfg.ARC_SEGMENT_MAX_RESIDUAL_PX:
                continue
            if span_deg < cfg.ARC_SEGMENT_MIN_SPAN_DEG:
                continue

            arc_segments.append({
                'id': idx,
                'points': pts,
                'length_px': arc_len,
                'ellipse_hint': ellipse_hint,
                'coverage': coverage,
                'span_deg': span_deg,
                'residual_px': residual_px,
            })

        line_segments = []
        lines = cv2.HoughLinesP(
            edges_filtered,
            1,
            np.pi / 180.0,
            threshold=cfg.HOUGH_LINE_THRESHOLD,
            minLineLength=cfg.HOUGH_MIN_LINE_LENGTH,
            maxLineGap=cfg.HOUGH_MAX_LINE_GAP
        )
        if lines is not None:
            for idx, line in enumerate(lines[: cfg.RIM_CANDIDATE_PAIR_TOPK * 2]):
                x1, y1, x2, y2 = line[0]
                pts = np.array([[x1, y1], [x2, y2]], dtype=np.float32)
                length_px = float(np.linalg.norm(pts[1] - pts[0]))
                if length_px < cfg.HOUGH_MIN_LINE_LENGTH:
                    continue

                angle_deg = float(math.degrees(math.atan2(y2 - y1, x2 - x1)))
                line_segments.append({
                    'id': idx,
                    'points': pts,
                    'length_px': length_px,
                    'angle_deg': angle_deg,
                })

        if not arc_segments and np.count_nonzero(boundary_band) > 0:
            boundary_contours, _ = cv2.findContours(boundary_band, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            for idx, cnt in enumerate(boundary_contours):
                pts = cnt.reshape(-1, 2).astype(np.float32)
                if len(pts) < cfg.ARC_SEGMENT_MIN_POINTS:
                    continue
                ellipse_hint = self._fit_ellipse_from_points(pts)
                if ellipse_hint is None:
                    continue
                arc_segments.append({
                    'id': 1000 + idx,
                    'points': pts,
                    'length_px': float(cv2.arcLength(cnt, False)),
                    'ellipse_hint': ellipse_hint,
                    'coverage': self._estimate_angular_coverage(pts, ellipse_hint),
                    'span_deg': self._estimate_angular_coverage(pts, ellipse_hint) * 360.0,
                    'residual_px': 0.0,
                })

        return arc_segments, line_segments

    def _score_candidate(self, ellipse, support_cloud, rim_mask, boundary_band, segment_ids, source_label):
        if ellipse is None:
            return None

        MA, ma = ellipse[1]
        if MA <= 1e-6 or ma <= 1e-6:
            return None
        if ma / MA < cfg.MIN_AXIS_RATIO:
            return None

        support_pts = np.asarray(support_cloud, dtype=np.float32).reshape(-1, 2)
        if len(support_pts) < 5:
            return None

        avg_radius = (MA + ma) / 4.0
        residual_px_all = np.abs(self._ellipse_norm_distance(support_pts, ellipse) - 1.0) * avg_radius
        inlier_mask = residual_px_all <= cfg.CANDIDATE_INLIER_THRESH_PX
        inlier_points = support_pts[inlier_mask]
        if len(inlier_points) < 5:
            return None

        residual_px = float(np.mean(residual_px_all[inlier_mask]))
        support_ratio = float(len(inlier_points) / max(1, len(support_pts)))
        coverage = self._estimate_angular_coverage(inlier_points, ellipse)
        mask_overlap, ring_ratio, fill_iou = self._ellipse_overlap_metrics(ellipse, rim_mask, boundary_band)
        centroid_score = self._mask_centroid_score(ellipse, rim_mask)
        segment_count = max(1, len(segment_ids))
        segment_diversity = float(np.clip(segment_count / 3.0, 0.0, 1.0))
        residual_score = float(np.clip(1.0 - residual_px / (cfg.ELLIPSE_RESIDUAL_MAX + 1e-6), 0.0, 1.0))

        score = (
            0.30 * support_ratio
            + 0.22 * coverage
            + 0.20 * mask_overlap
            + 0.14 * residual_score
            + 0.08 * segment_diversity
            + 0.06 * centroid_score
        )
        score = float(np.clip(score, 0.0, 1.0))

        reason = (
            f"support={support_ratio:.2f}, coverage={coverage:.2f}, "
            f"overlap={mask_overlap:.2f}, residual={residual_px:.2f}, src={source_label}"
        )

        return {
            'ellipse': ellipse,
            'score': score,
            'support_ratio': support_ratio,
            'coverage': coverage,
            'mask_overlap': float(mask_overlap),
            'ring_overlap': float(ring_ratio),
            'fill_iou': float(fill_iou),
            'residual_px': residual_px,
            'segment_ids': list(segment_ids),
            'segment_count': segment_count,
            'source': source_label,
            'inlier_points': inlier_points,
            'reason': reason,
        }

    def _deduplicate_candidates(self, candidates):
        unique = []
        for candidate in sorted(candidates, key=lambda item: item['score'], reverse=True):
            cx, cy = candidate['ellipse'][0]
            MA, ma = candidate['ellipse'][1]
            ang = candidate['ellipse'][2]

            duplicated = False
            for kept in unique:
                kx, ky = kept['ellipse'][0]
                kMA, kma = kept['ellipse'][1]
                kang = kept['ellipse'][2]
                center_tol = max(3.0, 0.05 * max(MA, kMA))
                axis_tol = max(4.0, 0.08 * max(MA, kMA))
                angle_diff = abs(ang - kang)
                angle_diff = min(angle_diff, 180.0 - angle_diff)
                if (
                    math.hypot(cx - kx, cy - ky) < center_tol
                    and abs(MA - kMA) < axis_tol
                    and abs(ma - kma) < axis_tol
                    and angle_diff < 12.0
                ):
                    duplicated = True
                    break

            if not duplicated:
                unique.append(candidate)
        return unique

    def generate_and_score_candidates(self, arc_segments, boundary_points, rim_mask, boundary_band):
        """从弧段中生成少量 ellipse candidates，并用 support / coverage / overlap 打分。"""
        arc_segments = sorted(arc_segments, key=lambda seg: seg['length_px'], reverse=True)
        support_cloud = None
        if arc_segments:
            support_cloud = np.vstack([seg['points'] for seg in arc_segments]).astype(np.float32)
        elif len(boundary_points) >= 5:
            support_cloud = np.asarray(boundary_points, dtype=np.float32).reshape(-1, 2)

        if support_cloud is None or len(support_cloud) < 5:
            return [], None

        specs = []
        top_arcs = arc_segments[: cfg.RIM_CANDIDATE_PAIR_TOPK]
        for seg in top_arcs:
            specs.append({
                'source': f"arc_{seg['id']}",
                'segment_ids': [seg['id']],
                'points': seg['points'],
            })

        for i in range(len(top_arcs)):
            for j in range(i + 1, len(top_arcs)):
                pts = np.vstack([top_arcs[i]['points'], top_arcs[j]['points']]).astype(np.float32)
                specs.append({
                    'source': f"arc_pair_{top_arcs[i]['id']}_{top_arcs[j]['id']}",
                    'segment_ids': [top_arcs[i]['id'], top_arcs[j]['id']],
                    'points': pts,
                })

        if len(top_arcs) >= 2:
            specs.append({
                'source': "arc_union",
                'segment_ids': [seg['id'] for seg in top_arcs],
                'points': np.vstack([seg['points'] for seg in top_arcs]).astype(np.float32),
            })

        if len(boundary_points) >= 5:
            specs.append({
                'source': "mask_boundary",
                'segment_ids': [],
                'points': np.asarray(boundary_points, dtype=np.float32).reshape(-1, 2),
            })

        candidates = []
        for spec in specs:
            ellipse = self._fit_ellipse_from_points(spec['points'])
            candidate = self._score_candidate(
                ellipse,
                support_cloud,
                rim_mask,
                boundary_band,
                spec['segment_ids'],
                spec['source']
            )
            if candidate is not None and candidate['score'] >= cfg.CANDIDATE_SCORE_MIN:
                candidates.append(candidate)

        candidates = self._deduplicate_candidates(candidates)
        candidates = sorted(candidates, key=lambda item: item['score'], reverse=True)
        return candidates[: cfg.RIM_CANDIDATE_MAX_COUNT], support_cloud

    def refit_candidate_ellipse(self, support_cloud, seed_ellipse):
        """对最优 candidate 做一次基于 inlier 的 refit。"""
        if seed_ellipse is None:
            return None, np.empty((0, 2), dtype=np.float32)
        if support_cloud is None or len(support_cloud) < 5:
            return seed_ellipse, np.empty((0, 2), dtype=np.float32)

        pts = np.asarray(support_cloud, dtype=np.float32).reshape(-1, 2)
        MA, ma = seed_ellipse[1]
        avg_radius = (MA + ma) / 4.0
        residual_px = np.abs(self._ellipse_norm_distance(pts, seed_ellipse) - 1.0) * avg_radius
        inlier_mask = residual_px <= (cfg.CANDIDATE_INLIER_THRESH_PX * 1.25)
        inliers = pts[inlier_mask]

        if len(inliers) < 5:
            return seed_ellipse, pts

        refined = self._fit_ellipse_from_points(inliers)
        if refined is None:
            return seed_ellipse, inliers

        return refined, inliers

    def fit_circle_or_ellipse_fallback(self, boundary_points, rim_mask, boundary_band):
        """第一阶段兜底：用 mask boundary 直接做受限拟合，但不再走 convexHull 主链路。"""
        pts = np.asarray(boundary_points, dtype=np.float32).reshape(-1, 2)
        if len(pts) < 3:
            return None

        ellipse = self._fit_ellipse_from_points(pts)
        if ellipse is None:
            (cx, cy), radius = cv2.minEnclosingCircle(pts)
            ellipse = ((float(cx), float(cy)), (float(radius * 2.0), float(radius * 2.0)), 0.0)

        return self._score_candidate(ellipse, pts, rim_mask, boundary_band, [], "fallback_mask_boundary")

    def detect_body_side_lines(self, line_segments, body_bbox=None):
        """为第二阶段预留接口，本阶段仅保留占位输出。"""
        _ = line_segments
        _ = body_bbox
        return []

    def _body_bbox_geometry(self, body_bbox):
        if body_bbox is None:
            return None

        cx, cy, bw, bh = [float(v) for v in body_bbox]
        body_dir = np.array([0.0, 1.0], dtype=np.float32) if bh >= bw else np.array([1.0, 0.0], dtype=np.float32)
        aspect = max(bw, bh) / max(1e-6, min(bw, bh))
        axis_conf = float(np.clip((aspect - cfg.POSE_BODY_AXIS_ASPECT_MIN) / 0.80, 0.0, 1.0))
        return {
            'center_uv': np.array([cx, cy], dtype=np.float32),
            'body_dir': body_dir,
            'axis_conf': axis_conf,
            'bw': bw,
            'bh': bh,
        }

    def _ellipse_minor_axis_direction(self, angle_deg):
        phi = math.radians(angle_deg + 90.0)
        return np.array([math.cos(phi), math.sin(phi)], dtype=np.float32)

    def _compute_center_shift_px(self, MA, ma):
        axis_ratio = float(ma / max(MA, 1e-6))
        if axis_ratio >= cfg.NEAR_CIRCLE_AXIS_RATIO:
            return 0.0

        shift_px = cfg.POSE_CENTER_SHIFT_GAIN * MA * max(0.0, 1.0 - axis_ratio)
        shift_limit = cfg.POSE_CENTER_SHIFT_MAX_RATIO * ma
        return float(np.clip(shift_px, 0.0, shift_limit))

    def _apply_hemisphere_convention(self, normal, center_3d):
        normal = np.asarray(normal, dtype=np.float32)
        center_3d = np.asarray(center_3d, dtype=np.float32)
        dot_view = float(np.dot(normal, center_3d))
        if cfg.NORMAL_POINT_AWAY_FROM_CAMERA:
            if dot_view < 0.0:
                normal = -normal
        else:
            if dot_view > 0.0:
                normal = -normal
        normal /= (np.linalg.norm(normal) + 1e-6)
        return normal

    def _build_pose_candidate_from_ellipse(
        self,
        local_ellipse,
        roi_offset,
        sign,
        center_shift_px,
        candidate_id,
        depth_prior_m=None,
        depth_prior_confidence=0.0,
        frontal_mode=False
    ):
        MA, ma = local_ellipse[1]
        if MA <= 0.0 or ma <= 0.0:
            return None

        axis_ratio = float(ma / max(MA, 1e-6))
        real_angle = float(local_ellipse[2])
        minor_axis_dir = self._ellipse_minor_axis_direction(real_angle)
        local_center_uv = np.array(local_ellipse[0], dtype=np.float32) + minor_axis_dir * float(sign) * float(center_shift_px)
        global_center_uv = local_center_uv + np.array([roi_offset[0], roi_offset[1]], dtype=np.float32)

        Z_ellipse = float(self.fx * cfg.REAL_TUBE_DIAMETER / MA)
        X = (float(global_center_uv[0]) - self.cx) * Z_ellipse / self.fx
        Y = (float(global_center_uv[1]) - self.cy) * Z_ellipse / self.fy
        center_3d = np.array([X, Y, Z_ellipse], dtype=np.float32)

        tilt = 0.0 if frontal_mode else math.acos(min(1.0, max(-1.0, axis_ratio)))
        nx = float(sign) * math.sin(tilt) * float(minor_axis_dir[0])
        ny = float(sign) * math.sin(tilt) * float(minor_axis_dir[1])
        nz = -math.cos(tilt)
        normal = self._apply_hemisphere_convention(np.array([nx, ny, nz], dtype=np.float32), center_3d)

        axis_img_dir = np.array([normal[0] * self.fx, normal[1] * self.fy], dtype=np.float32)
        if np.linalg.norm(axis_img_dir) < 1e-6:
            axis_img_dir = minor_axis_dir * float(sign)
        axis_img_dir /= (np.linalg.norm(axis_img_dir) + 1e-6)

        return {
            'candidate_id': int(candidate_id),
            'center_3d': center_3d,
            'normal': normal,
            'center_uv': global_center_uv,
            'local_center_uv': local_center_uv,
            'axis_img_dir': axis_img_dir,
            'axis_ratio': axis_ratio,
            'ellipse_2d': (
                (float(global_center_uv[0]), float(global_center_uv[1])),
                (float(MA), float(ma)),
                real_angle
            ),
            'z_ellipse': Z_ellipse,
            'z_prior': None if depth_prior_m is None else float(depth_prior_m),
            'z_fused': Z_ellipse,
            'z_fusion_weight': 0.0,
            'depth_prior_confidence': float(np.clip(depth_prior_confidence, 0.0, 1.0)),
            'near_circle_degenerate': bool(axis_ratio >= cfg.NEAR_CIRCLE_AXIS_RATIO),
            'depth_prior_ignored': True,
            'observability_state': 'frontal' if frontal_mode else 'weak_side',
            'center_shift_px': float(center_shift_px),
            'solver_stage': 'phase2_single_circle_candidates',
        }

    def solve_circle_pose_candidates(
        self,
        local_ellipse,
        roi_offset,
        body_bbox=None,
        depth_prior_m=None,
        depth_prior_confidence=0.0
    ):
        """基于当前椭圆生成单圆双候选解；near-circle 时退化为 frontal 单候选。"""
        _ = body_bbox
        MA, ma = local_ellipse[1]
        if MA <= 0.0 or ma <= 0.0:
            return []

        axis_ratio = float(ma / max(MA, 1e-6))
        near_circle = axis_ratio >= cfg.NEAR_CIRCLE_AXIS_RATIO
        center_shift_px = 0.0 if near_circle else self._compute_center_shift_px(MA, ma)

        candidates = []
        if near_circle:
            candidate = self._build_pose_candidate_from_ellipse(
                local_ellipse,
                roi_offset,
                sign=1.0,
                center_shift_px=0.0,
                candidate_id=0,
                depth_prior_m=depth_prior_m,
                depth_prior_confidence=depth_prior_confidence,
                frontal_mode=True
            )
            if candidate is not None:
                candidates.append(candidate)
            return candidates

        for candidate_id, sign in enumerate((1.0, -1.0)):
            candidate = self._build_pose_candidate_from_ellipse(
                local_ellipse,
                roi_offset,
                sign=sign,
                center_shift_px=center_shift_px,
                candidate_id=candidate_id,
                depth_prior_m=depth_prior_m,
                depth_prior_confidence=depth_prior_confidence,
                frontal_mode=False
            )
            if candidate is not None:
                candidates.append(candidate)
        return candidates

    def score_candidates_rgb_only(self, pose_candidates, body_bbox=None):
        """第二阶段第一步：仅用当前 RGB 几何和 body bbox 做双候选消歧。"""
        if not pose_candidates:
            return [], None

        body_geom = self._body_bbox_geometry(body_bbox)
        scored_candidates = []

        for candidate in pose_candidates:
            cand = dict(candidate)
            if body_geom is None:
                cand['body_axis_score'] = 0.5
                cand['body_center_score'] = 0.5
                cand['centerline_score'] = 0.5
                cand['rgb_only_score'] = 0.5
                cand['rgb_only_reason'] = 'no body bbox, keep neutral score'
                scored_candidates.append(cand)
                continue

            axis_img_dir = np.asarray(cand.get('axis_img_dir', body_geom['body_dir']), dtype=np.float32)
            axis_norm = float(np.linalg.norm(axis_img_dir))
            if axis_norm < 1e-6:
                axis_img_dir = body_geom['body_dir'].copy()
            else:
                axis_img_dir /= axis_norm

            body_dir = body_geom['body_dir']
            body_center = body_geom['center_uv']
            center_uv = np.asarray(cand['center_uv'], dtype=np.float32)

            body_axis_align = abs(float(np.dot(axis_img_dir, body_dir)))
            body_axis_score = 0.5 + 0.5 * body_axis_align

            body_center_vec = body_center - center_uv
            body_center_norm = float(np.linalg.norm(body_center_vec))
            if body_center_norm < 1e-6:
                body_center_score = 0.5
            else:
                body_center_vec /= body_center_norm
                body_center_score = float(np.clip(
                    0.5 + 0.5 * float(np.dot(axis_img_dir, body_center_vec)),
                    0.0,
                    1.0
                ))

            normal_to_body = np.array([-body_dir[1], body_dir[0]], dtype=np.float32)
            perp_dist = abs(float(np.dot(center_uv - body_center, normal_to_body)))
            half_span = max(6.0, 0.5 * min(body_geom['bw'], body_geom['bh']))
            centerline_score = float(np.clip(1.0 - perp_dist / (half_span + 1e-6), 0.0, 1.0))

            geometric_score = (
                cfg.POSE_RGB_BODY_CENTER_WEIGHT * body_center_score
                + cfg.POSE_RGB_CENTERLINE_WEIGHT * centerline_score
                + cfg.POSE_RGB_BODY_AXIS_WEIGHT * body_axis_score
            )
            axis_conf = body_geom['axis_conf']
            rgb_only_score = float((1.0 - axis_conf) * 0.5 + axis_conf * geometric_score)

            cand['body_axis_score'] = float(body_axis_score)
            cand['body_center_score'] = float(body_center_score)
            cand['centerline_score'] = float(centerline_score)
            cand['rgb_only_score'] = float(np.clip(rgb_only_score, 0.0, 1.0))
            cand['rgb_only_reason'] = (
                f"rgb={cand['rgb_only_score']:.2f}, body_center={body_center_score:.2f}, "
                f"centerline={centerline_score:.2f}, body_axis={body_axis_score:.2f}"
            )
            scored_candidates.append(cand)

        scored_candidates = sorted(
            scored_candidates,
            key=lambda item: item.get('rgb_only_score', 0.0),
            reverse=True
        )
        return scored_candidates, dict(scored_candidates[0])

    def frontal_fallback(self, pose_result):
        """near-circle 情况下只保守输出 center + optical-axis-like axis，并显式打标。"""
        pose_result['frontal_fallback_ready'] = True
        pose_result['observability_state'] = 'frontal'
        pose_result['low_observability'] = True
        return pose_result

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
                'q_overlap': 0.0,
                'consistency_score': 0.0,
            }

        MA = pose_dict['ellipse_2d'][1][0]
        ma = pose_dict['ellipse_2d'][1][1]
        area = math.pi * (MA / 2.0) * (ma / 2.0)

        if area < cfg.ELLIPSE_AREA_MIN_PX2:
            q_area = float(np.clip(area / (cfg.ELLIPSE_AREA_MIN_PX2 + 1e-6), 0.0, 1.0))
        elif area > cfg.ELLIPSE_AREA_MAX_PX2:
            q_area = float(np.clip(
                (2.0 * cfg.ELLIPSE_AREA_MAX_PX2 - area) / (cfg.ELLIPSE_AREA_MAX_PX2 + 1e-6),
                0.0,
                1.0
            ))
        else:
            q_area = 1.0

        coverage = float(ellipse_metrics.get('coverage', 0.0))
        q_coverage = float(np.clip(coverage / (cfg.RIM_COVERAGE_MIN + 1e-6), 0.0, 1.0))

        residual = float(ellipse_metrics.get('residual', cfg.ELLIPSE_RESIDUAL_MAX * 2.0))
        q_residual = float(np.clip(1.0 - residual / (cfg.ELLIPSE_RESIDUAL_MAX + 1e-6), 0.0, 1.0))

        axis_ratio = float(pose_dict.get('axis_ratio', 0.0))
        if cfg.AXIS_RATIO_QUALITY_MIN <= axis_ratio <= cfg.AXIS_RATIO_QUALITY_MAX:
            q_axis = 1.0
        elif axis_ratio < cfg.AXIS_RATIO_QUALITY_MIN:
            q_axis = float(np.clip(axis_ratio / (cfg.AXIS_RATIO_QUALITY_MIN + 1e-6), 0.0, 1.0))
        else:
            span = max(1e-6, 1.0 - cfg.AXIS_RATIO_QUALITY_MAX)
            q_axis = float(np.clip((1.0 - axis_ratio) / span, 0.0, 1.0))

        mask_overlap = float(ellipse_metrics.get('mask_overlap', 0.0))
        q_overlap = float(np.clip(mask_overlap / 0.65, 0.0, 1.0))

        q = (
            0.18 * q_area
            + 0.22 * q_coverage
            + 0.28 * q_residual
            + 0.18 * q_axis
            + 0.14 * q_overlap
        )

        consistency_score = float(np.clip(ellipse_metrics.get('consistency_score', 1.0), 0.0, 1.0))
        q *= (0.7 + 0.3 * consistency_score)
        q = float(np.clip(q, 0.0, 1.0))

        return q, {
            'q_area': q_area,
            'q_coverage': q_coverage,
            'q_residual': q_residual,
            'q_axis': q_axis,
            'q_overlap': q_overlap,
            'consistency_score': consistency_score,
            'coverage': coverage,
            'residual_px': residual,
            'mask_overlap': mask_overlap,
            'support_ratio': float(ellipse_metrics.get('support_ratio', 0.0)),
            'ellipse_area_px2': area,
        }

    def _build_debug_visualization(
        self,
        roi_img,
        boundary_band,
        edges_filtered,
        arc_segments,
        line_segments,
        candidates,
        selected_candidate
    ):
        debug_vis = roi_img.copy()

        band_overlay = debug_vis.copy()
        band_overlay[boundary_band > 0] = (0, 180, 0)
        debug_vis = cv2.addWeighted(debug_vis, 0.80, band_overlay, 0.20, 0.0)
        debug_vis[edges_filtered > 0] = (255, 120, 0)

        for seg in arc_segments:
            pts = seg['points'].astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(debug_vis, [pts], False, (0, 220, 255), 1)

        for line in line_segments:
            p0 = tuple(line['points'][0].astype(int))
            p1 = tuple(line['points'][1].astype(int))
            cv2.line(debug_vis, p0, p1, (255, 255, 0), 1)

        palette = [
            (255, 0, 255),
            (0, 255, 0),
            (255, 255, 0),
            (0, 128, 255),
        ]
        for idx, candidate in enumerate(candidates[: cfg.DEBUG_TOP_CANDIDATES]):
            color = palette[idx % len(palette)]
            cv2.ellipse(debug_vis, candidate['ellipse'], color, 1)
            cx, cy = candidate['ellipse'][0]
            cv2.putText(
                debug_vis,
                f"C{idx}:{candidate['score']:.2f}",
                (int(cx) + 4, int(cy) - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.40,
                color,
                1,
                cv2.LINE_AA
            )

        if selected_candidate is not None:
            cv2.ellipse(debug_vis, selected_candidate['ellipse'], (0, 0, 255), 2)

        info_lines = [
            f"band={int(np.count_nonzero(boundary_band))} arc={len(arc_segments)} line={len(line_segments)}"
        ]
        for idx, candidate in enumerate(candidates[: cfg.DEBUG_TOP_CANDIDATES]):
            info_lines.append(
                f"{idx}:{candidate['source']} s={candidate['score']:.2f} "
                f"sup={candidate['support_ratio']:.2f} cov={candidate['coverage']:.2f}"
            )
        if selected_candidate is not None:
            info_lines.append(f"pick: {selected_candidate.get('selection_reason', selected_candidate['reason'])}")

        y = 18
        for line in info_lines[:6]:
            cv2.putText(
                debug_vis,
                line[:72],
                (8, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (255, 255, 255),
                1,
                cv2.LINE_AA
            )
            y += 16

        return debug_vis

    def _finalize_pose_result_from_candidate(self, pose_candidate):
        if pose_candidate is None:
            return None, "No Pose Candidate"

        pose_result = {
            'center_3d': np.asarray(pose_candidate['center_3d'], dtype=np.float32),
            'normal': np.asarray(pose_candidate['normal'], dtype=np.float32),
            'ellipse_2d': pose_candidate['ellipse_2d'],
            'axis_ratio': float(pose_candidate['axis_ratio']),
            'z_ellipse': float(pose_candidate['z_ellipse']),
            'z_prior': pose_candidate.get('z_prior', None),
            'z_fused': float(pose_candidate['z_fused']),
            'z_fusion_weight': float(pose_candidate.get('z_fusion_weight', 0.0)),
            'depth_prior_confidence': float(pose_candidate.get('depth_prior_confidence', 0.0)),
            'near_circle_degenerate': bool(pose_candidate.get('near_circle_degenerate', False)),
            'depth_prior_ignored': bool(pose_candidate.get('depth_prior_ignored', True)),
            'observability_state': str(pose_candidate.get('observability_state', 'weak_side')),
            'selected_pose_candidate_id': int(pose_candidate['candidate_id']),
            'selected_pose_reason': str(pose_candidate.get('rgb_only_reason', 'selected by pose candidate score')),
            'center_shift_px': float(pose_candidate.get('center_shift_px', 0.0)),
            'low_observability': False,
        }
        if pose_result['near_circle_degenerate']:
            pose_result = self.frontal_fallback(pose_result)
        return pose_result, "Success"

    def _overlay_pose_debug_text(self, debug_vis_img, pose_result):
        if debug_vis_img is None or pose_result is None:
            return debug_vis_img

        lines = [
            f"mode={pose_result.get('observability_state', '?')} pose={int(pose_result.get('selected_pose_candidate_id', -1))} "
            f"ar={float(pose_result.get('axis_ratio', 0.0)):.2f}"
        ]
        for cand in pose_result.get('pose_candidates_3d', [])[:2]:
            lines.append(
                f"P{int(cand.get('candidate_id', -1))}: s={float(cand.get('rgb_only_score', 0.0)):.2f} "
                f"bc={float(cand.get('body_center_score', 0.0)):.2f} "
                f"cl={float(cand.get('centerline_score', 0.0)):.2f}"
            )
        lines.append(str(pose_result.get('selected_pose_reason', ''))[:78])

        y = max(18, debug_vis_img.shape[0] - 18 * len(lines) - 8)
        for line in lines:
            cv2.putText(
                debug_vis_img,
                line,
                (8, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (180, 255, 180),
                1,
                cv2.LINE_AA
            )
            y += 16
        return debug_vis_img

    def process_detection(
        self,
        image,
        precise_rim_mask,
        crop_offset,
        body_bbox=None,
        depth_prior_m=None,
        depth_prior_confidence=0.0
    ):
        """第一阶段主流程：boundary band -> arc segments -> candidates -> refit."""
        _ = crop_offset
        ys, xs = np.where(precise_rim_mask > 0)
        if len(ys) == 0:
            return None, "Empty Mask", [], None, (0, 0)

        min_x, max_x = np.min(xs), np.max(xs)
        min_y, max_y = np.min(ys), np.max(ys)
        pad = 20
        x1 = max(0, min_x - pad)
        y1 = max(0, min_y - pad)
        x2 = min(image.shape[1], max_x + pad)
        y2 = min(image.shape[0], max_y + pad)

        roi_img = image[y1:y2, x1:x2]
        roi_mask = precise_rim_mask[y1:y2, x1:x2]
        if roi_img.size == 0:
            return None, "Empty ROI", [], None, (x1, y1)

        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.bilateralFilter(gray, 5, 50, 50)
        enhanced = self.clahe.apply(gray_blur)

        rim_mask, boundary_band, boundary_contours, _ = self.extract_rim_boundary_band(roi_mask)
        if rim_mask is None or boundary_band is None:
            return None, "Invalid Rim Mask", [], roi_img.copy(), (x1, y1)

        edges = self.auto_canny(enhanced)
        edges_filtered = cv2.bitwise_and(edges, edges, mask=boundary_band)
        arc_segments, line_segments = self.detect_arc_support_segments(edges_filtered, boundary_band)

        boundary_points = np.empty((0, 2), dtype=np.float32)
        if boundary_contours:
            boundary_points = np.vstack([cnt.reshape(-1, 2) for cnt in boundary_contours]).astype(np.float32)

        candidates, support_cloud = self.generate_and_score_candidates(
            arc_segments,
            boundary_points,
            rim_mask,
            boundary_band
        )

        selected_candidate = None
        if candidates:
            selected_candidate = dict(candidates[0])
            refined_ellipse, refit_inliers = self.refit_candidate_ellipse(support_cloud, selected_candidate['ellipse'])
            refined_candidate = self._score_candidate(
                refined_ellipse,
                support_cloud,
                rim_mask,
                boundary_band,
                selected_candidate['segment_ids'],
                f"{selected_candidate['source']}_refit"
            )
            if refined_candidate is not None:
                refined_candidate['selection_reason'] = (
                    f"best candidate refit {selected_candidate['score']:.3f}->{refined_candidate['score']:.3f}"
                )
                if refined_candidate['score'] >= selected_candidate['score'] - 0.02:
                    refined_candidate['inlier_points'] = refit_inliers
                    selected_candidate = refined_candidate
                else:
                    selected_candidate['selection_reason'] = (
                        f"keep pre-refit score {selected_candidate['score']:.3f} > {refined_candidate['score']:.3f}"
                    )
            else:
                selected_candidate['selection_reason'] = f"best candidate score {selected_candidate['score']:.3f}"
        else:
            selected_candidate = self.fit_circle_or_ellipse_fallback(boundary_points, rim_mask, boundary_band)
            if selected_candidate is not None:
                selected_candidate['selection_reason'] = "fallback to mask-boundary fit"
                candidates = [selected_candidate]

        debug_vis_img = self._build_debug_visualization(
            roi_img,
            boundary_band,
            edges_filtered,
            arc_segments,
            line_segments,
            candidates,
            selected_candidate
        )

        if selected_candidate is None:
            return None, "No Valid Candidate", [], debug_vis_img, (x1, y1)

        final_ellipse = selected_candidate['ellipse']
        pose_candidates = self.solve_circle_pose_candidates(
            final_ellipse,
            (x1, y1),
            body_bbox=body_bbox,
            depth_prior_m=depth_prior_m,
            depth_prior_confidence=depth_prior_confidence
        )
        pose_candidates_scored, best_pose_candidate = self.score_candidates_rgb_only(
            pose_candidates,
            body_bbox=body_bbox
        )
        pose_result, status = self._finalize_pose_result_from_candidate(best_pose_candidate)
        if pose_result is None:
            return None, status, self._summarize_candidates(candidates), debug_vis_img, (x1, y1)

        local_support_points = selected_candidate.get('inlier_points', np.empty((0, 2), dtype=np.float32))
        if len(local_support_points) < 5:
            local_support_points = support_cloud if support_cloud is not None else boundary_points
        local_support_points = np.asarray(local_support_points, dtype=np.float32).reshape(-1, 2)

        MA, ma = final_ellipse[1]
        avg_radius = (MA + ma) / 4.0
        norm_dist_local = self._ellipse_norm_distance(local_support_points, final_ellipse)
        residual_px = float(np.mean(np.abs(norm_dist_local - 1.0)) * avg_radius)
        coverage = float(selected_candidate.get('coverage', 0.0))
        support_ratio = float(selected_candidate.get('support_ratio', 0.0))
        mask_overlap = float(selected_candidate.get('mask_overlap', 0.0))
        perimeter = self._ellipse_perimeter(MA, ma)
        coverage_from_points = float(min(1.0, len(local_support_points) / (perimeter + 1e-6)))
        coverage = max(coverage, coverage_from_points)
        consistency_score = self.check_body_rim_consistency(
            body_bbox,
            pose_result['ellipse_2d'],
            pose_result['normal']
        )

        fit_residual_score = float(np.clip(
            1.0 - residual_px / (cfg.ELLIPSE_RESIDUAL_MAX + 1e-6),
            0.0,
            1.0
        ))
        ellipse_fit_score = float(np.clip(
            0.40 * fit_residual_score + 0.30 * coverage + 0.30 * mask_overlap,
            0.0,
            1.0
        ))

        metrics = {
            'coverage': coverage,
            'residual': residual_px,
            'mask_overlap': mask_overlap,
            'support_ratio': support_ratio,
            'consistency_score': consistency_score,
            'ellipse_fit_score': ellipse_fit_score,
        }
        quality_score, quality_components = self.compute_quality_score(pose_result, metrics)

        side_lines = self.detect_body_side_lines(line_segments, body_bbox=body_bbox)
        pose_result['quality_score'] = quality_score
        pose_result['quality_components'] = quality_components
        pose_result['consistency_score'] = consistency_score
        pose_result['ellipse_fit_score'] = ellipse_fit_score
        pose_result['mask_overlap'] = mask_overlap
        pose_result['support_ratio'] = support_ratio
        pose_result['rim_pipeline_stage'] = 'phase2_pose_candidate_pipeline'
        pose_result['rim_candidates'] = self._summarize_candidates(candidates)
        pose_result['selected_candidate_reason'] = selected_candidate.get(
            'selection_reason',
            selected_candidate['reason']
        )
        pose_result['side_line_count'] = len(side_lines)
        if len(side_lines) >= 2 and not pose_result.get('near_circle_degenerate', False):
            pose_result['observability_state'] = 'oblique'
        elif not pose_result.get('near_circle_degenerate', False):
            pose_result['observability_state'] = 'weak_side'
        pose_result['pose_candidates_3d'] = [
            {
                'candidate_id': int(cand['candidate_id']),
                'center_3d': [float(x) for x in cand['center_3d']],
                'axis_3d': [float(x) for x in cand['normal']],
                'center_uv': [float(cand['center_uv'][0]), float(cand['center_uv'][1])],
                'axis_ratio': float(cand['axis_ratio']),
                'center_shift_px': float(cand.get('center_shift_px', 0.0)),
                'rgb_only_score': float(cand.get('rgb_only_score', 0.0)),
                'body_axis_score': float(cand.get('body_axis_score', 0.0)),
                'body_center_score': float(cand.get('body_center_score', 0.0)),
                'centerline_score': float(cand.get('centerline_score', 0.0)),
                'observability_state': str(cand.get('observability_state', pose_result['observability_state'])),
                'solver_stage': str(cand.get('solver_stage', 'phase2_single_circle_candidates')),
                'reason': str(cand.get('rgb_only_reason', '')),
            }
            for cand in pose_candidates_scored
        ]
        debug_vis_img = self._overlay_pose_debug_text(debug_vis_img, pose_result)

        return pose_result, "Success", pose_result['rim_candidates'], debug_vis_img, (x1, y1)
