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

    def _canonicalize_2d_direction(self, direction):
        vec = np.asarray(direction, dtype=np.float32).reshape(2)
        norm = float(np.linalg.norm(vec))
        if norm < 1e-6:
            return None

        vec = vec / norm
        if abs(float(vec[0])) >= abs(float(vec[1])):
            if float(vec[0]) < 0.0:
                vec = -vec
        else:
            if float(vec[1]) < 0.0:
                vec = -vec
        return vec.astype(np.float32)

    def _body_bbox_geometry(self, body_bbox):
        if body_bbox is None:
            return None

        cx, cy, bw, bh = [float(v) for v in body_bbox]
        body_dir = np.array([0.0, 1.0], dtype=np.float32) if bh >= bw else np.array([1.0, 0.0], dtype=np.float32)
        body_dir = self._canonicalize_2d_direction(body_dir)
        if body_dir is None:
            return None

        short_span = max(1.0, min(bw, bh))
        long_span = max(1.0, max(bw, bh))
        aspect = long_span / short_span
        axis_conf = float(np.clip((aspect - cfg.POSE_BODY_AXIS_ASPECT_MIN) / 0.80, 0.0, 1.0))
        normal_dir = np.array([-body_dir[1], body_dir[0]], dtype=np.float32)
        return {
            'center_uv': np.array([cx, cy], dtype=np.float32),
            'body_dir': body_dir,
            'normal_dir': normal_dir,
            'axis_conf': axis_conf,
            'bw': bw,
            'bh': bh,
            'long_span': long_span,
            'short_span': short_span,
            'axis_range': (-0.5 * long_span, 0.5 * long_span),
            'normal_range': (-0.5 * short_span, 0.5 * short_span),
            'side_offsets': (-0.5 * short_span, 0.5 * short_span),
            'source': 'bbox_fallback',
        }

    def _estimate_body_geometry(self, body_mask=None, body_bbox=None):
        if body_mask is not None:
            mask = (body_mask > 0).astype(np.uint8)
            if np.count_nonzero(mask) >= cfg.BODY_SIDE_MIN_POINTS:
                ys, xs = np.where(mask > 0)
                pts = np.stack([xs, ys], axis=1).astype(np.float32)
                center_uv = np.mean(pts, axis=0)
                centered = pts - center_uv
                cov = (centered.T @ centered) / max(1, len(centered) - 1)
                try:
                    eigvals, eigvecs = np.linalg.eigh(cov)
                except np.linalg.LinAlgError:
                    eigvals, eigvecs = None, None

                if eigvals is not None and eigvecs is not None:
                    axis_dir = self._canonicalize_2d_direction(eigvecs[:, int(np.argmax(eigvals))])
                    if axis_dir is not None:
                        normal_dir = np.array([-axis_dir[1], axis_dir[0]], dtype=np.float32)
                        axis_proj = centered @ axis_dir
                        normal_proj = centered @ normal_dir
                        axis_lo, axis_hi = np.percentile(axis_proj, [1.0, 99.0])
                        normal_lo, normal_hi = np.percentile(normal_proj, [1.0, 99.0])
                        long_span = float(max(axis_hi - axis_lo, 1.0))
                        short_span = float(max(normal_hi - normal_lo, 1.0))
                        aspect = long_span / max(short_span, 1e-6)
                        axis_conf = float(np.clip((aspect - cfg.POSE_BODY_AXIS_ASPECT_MIN) / 0.80, 0.0, 1.0))
                        edge_q = float(np.clip(cfg.BODY_SIDE_EDGE_QUANTILE, 0.02, 0.20))
                        side_offsets = (
                            float(np.quantile(normal_proj, edge_q)),
                            float(np.quantile(normal_proj, 1.0 - edge_q)),
                        )
                        return {
                            'center_uv': center_uv.astype(np.float32),
                            'body_dir': axis_dir,
                            'normal_dir': normal_dir,
                            'axis_conf': axis_conf,
                            'bw': float(body_bbox[2]) if body_bbox is not None else long_span,
                            'bh': float(body_bbox[3]) if body_bbox is not None else short_span,
                            'long_span': long_span,
                            'short_span': short_span,
                            'axis_range': (float(axis_lo), float(axis_hi)),
                            'normal_range': (float(normal_lo), float(normal_hi)),
                            'side_offsets': side_offsets,
                            'source': 'mask_pca',
                        }

        return self._body_bbox_geometry(body_bbox)

    def _offset_body_geometry(self, body_geom, offset_xy):
        if body_geom is None:
            return None

        shifted = {}
        for key, value in body_geom.items():
            if isinstance(value, np.ndarray):
                shifted[key] = value.copy()
            else:
                shifted[key] = value
        shifted['center_uv'] = np.asarray(body_geom['center_uv'], dtype=np.float32) + np.array(offset_xy, dtype=np.float32)
        return shifted

    def _build_side_search_bands(self, body_mask, body_geom):
        mask = (body_mask > 0).astype(np.uint8) * 255
        if np.count_nonzero(mask) < cfg.BODY_SIDE_MIN_POINTS or body_geom is None:
            return None

        h, w = mask.shape[:2]
        center_uv = np.asarray(body_geom['center_uv'], dtype=np.float32)
        body_dir = np.asarray(body_geom['body_dir'], dtype=np.float32)
        normal_dir = np.asarray(body_geom['normal_dir'], dtype=np.float32)

        yy, xx = np.indices((h, w), dtype=np.float32)
        dx = xx - center_uv[0]
        dy = yy - center_uv[1]
        axis_proj = dx * body_dir[0] + dy * body_dir[1]
        normal_proj = dx * normal_dir[0] + dy * normal_dir[1]

        axis_half = 0.5 * float(body_geom['long_span'])
        axis_trim = float(np.clip(cfg.BODY_SIDE_AXIS_TRIM_RATIO, 0.0, 0.35)) * float(body_geom['long_span'])
        axis_limit = max(4.0, axis_half - axis_trim)
        axis_gate = np.abs(axis_proj) <= axis_limit

        band_half_width = float(np.clip(
            float(body_geom['short_span']) * cfg.BODY_SIDE_BAND_WIDTH_RATIO,
            cfg.BODY_SIDE_BAND_WIDTH_MIN,
            cfg.BODY_SIDE_BAND_WIDTH_MAX
        ))
        inner_allowance = float(np.clip(
            float(body_geom['short_span']) * cfg.BODY_SIDE_INNER_ALLOWANCE_RATIO,
            cfg.BODY_SIDE_INNER_ALLOWANCE_MIN,
            cfg.BODY_SIDE_INNER_ALLOWANCE_MAX
        ))

        dilate_radius = max(1, int(round(band_half_width)))
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (2 * dilate_radius + 1, 2 * dilate_radius + 1)
        )
        near_body = cv2.dilate(mask, kernel, iterations=1)
        inside_dist = cv2.distanceTransform(mask, cv2.DIST_L2, 3)
        boundary_shell = (near_body > 0) & (((mask == 0) | (inside_dist <= inner_allowance)))

        left_offset, right_offset = body_geom['side_offsets']
        left_band = boundary_shell & axis_gate & (np.abs(normal_proj - left_offset) <= band_half_width)
        right_band = boundary_shell & axis_gate & (np.abs(normal_proj - right_offset) <= band_half_width)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        left_seed_points = np.empty((0, 2), dtype=np.float32)
        right_seed_points = np.empty((0, 2), dtype=np.float32)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            contour_pts = largest.reshape(-1, 2).astype(np.float32)
            contour_idx = np.round(contour_pts).astype(np.int32)
            contour_idx[:, 0] = np.clip(contour_idx[:, 0], 0, w - 1)
            contour_idx[:, 1] = np.clip(contour_idx[:, 1], 0, h - 1)
            left_seed_points = contour_pts[left_band[contour_idx[:, 1], contour_idx[:, 0]]]
            right_seed_points = contour_pts[right_band[contour_idx[:, 1], contour_idx[:, 0]]]

        return {
            'left_mask': (left_band.astype(np.uint8) * 255),
            'right_mask': (right_band.astype(np.uint8) * 255),
            'left_seed_points': left_seed_points,
            'right_seed_points': right_seed_points,
            'axis_proj': axis_proj,
            'normal_proj': normal_proj,
            'band_half_width_px': float(band_half_width),
            'inner_allowance_px': float(inner_allowance),
            'axis_limit_px': float(axis_limit),
        }

    def _refine_side_support_points(
        self,
        band_mask,
        edge_gate,
        grad_normal_abs,
        axis_proj,
        normal_proj,
        side_offset,
        label,
    ):
        band = band_mask > 0
        if np.count_nonzero(band) < 12:
            return np.empty((0, 2), dtype=np.float32), {
                'label': label,
                'reason': f"{label} band too small",
                'candidate_count': 0,
                'support_bin_count': 0,
                'grad_threshold': 0.0,
                'used_relaxed_gate': False,
            }

        band_grad = grad_normal_abs[band]
        if band_grad.size == 0:
            return np.empty((0, 2), dtype=np.float32), {
                'label': label,
                'reason': f"{label} band has no gradient sample",
                'candidate_count': 0,
                'support_bin_count': 0,
                'grad_threshold': 0.0,
                'used_relaxed_gate': False,
            }

        grad_thresh = max(
            float(cfg.BODY_SIDE_GRADIENT_MIN),
            float(np.percentile(band_grad, cfg.BODY_SIDE_GRADIENT_PERCENTILE))
        )
        candidate_mask = band & (grad_normal_abs >= grad_thresh) & (edge_gate > 0)
        used_relaxed_gate = False
        if np.count_nonzero(candidate_mask) < cfg.BODY_SIDE_SUPPORT_MIN_POINTS:
            candidate_mask = band & (grad_normal_abs >= grad_thresh)
            used_relaxed_gate = True

        ys, xs = np.where(candidate_mask)
        if len(xs) == 0:
            return np.empty((0, 2), dtype=np.float32), {
                'label': label,
                'reason': f"{label} no edge support after RGB refinement",
                'candidate_count': 0,
                'support_bin_count': 0,
                'grad_threshold': float(grad_thresh),
                'used_relaxed_gate': bool(used_relaxed_gate),
            }

        pts = np.stack([xs, ys], axis=1).astype(np.float32)
        t_vals = axis_proj[ys, xs]
        s_vals = normal_proj[ys, xs]
        g_vals = grad_normal_abs[ys, xs]

        bin_size = max(1.0, float(cfg.BODY_SIDE_BIN_SIZE_PX))
        t_origin = float(np.min(t_vals))
        bins = np.floor((t_vals - t_origin) / bin_size).astype(np.int32)
        chosen_indices = []
        for bin_id in np.unique(bins):
            local_ids = np.where(bins == bin_id)[0]
            if local_ids.size == 0:
                continue
            local_score = g_vals[local_ids] - 0.35 * np.abs(s_vals[local_ids] - side_offset)
            best_local = local_ids[int(np.argmax(local_score))]
            chosen_indices.append(best_local)

        if not chosen_indices:
            return np.empty((0, 2), dtype=np.float32), {
                'label': label,
                'reason': f"{label} support binning collapsed",
                'candidate_count': int(len(xs)),
                'support_bin_count': 0,
                'grad_threshold': float(grad_thresh),
                'used_relaxed_gate': bool(used_relaxed_gate),
            }

        chosen_indices = np.asarray(chosen_indices, dtype=np.int32)
        support_points = pts[chosen_indices]
        support_t = t_vals[chosen_indices]
        support_axis_span = float(np.max(support_t) - np.min(support_t)) if len(support_t) > 1 else 0.0

        info = {
            'label': label,
            'reason': '',
            'candidate_count': int(len(xs)),
            'support_bin_count': int(len(chosen_indices)),
            'grad_threshold': float(grad_thresh),
            'used_relaxed_gate': bool(used_relaxed_gate),
            'support_axis_span_px': float(support_axis_span),
        }
        if len(support_points) < cfg.BODY_SIDE_SUPPORT_MIN_POINTS:
            info['reason'] = f"{label} support points {len(support_points)}<{cfg.BODY_SIDE_SUPPORT_MIN_POINTS}"
        elif len(chosen_indices) < cfg.BODY_SIDE_SUPPORT_MIN_BINS:
            info['reason'] = f"{label} support bins {len(chosen_indices)}<{cfg.BODY_SIDE_SUPPORT_MIN_BINS}"

        return support_points, info

    def _fit_refined_side_line(self, points_xy, body_geom, label):
        pts = np.asarray(points_xy, dtype=np.float32).reshape(-1, 2)
        if len(pts) < cfg.BODY_SIDE_SUPPORT_MIN_POINTS or body_geom is None:
            return None

        body_dir = np.asarray(body_geom['body_dir'], dtype=np.float32)
        body_normal = np.asarray(body_geom['normal_dir'], dtype=np.float32)
        body_center = np.asarray(body_geom['center_uv'], dtype=np.float32)
        long_span = float(body_geom['long_span'])

        vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
        line_dir = np.array([float(vx), float(vy)], dtype=np.float32)
        line_dir = self._canonicalize_2d_direction(line_dir)
        if line_dir is None:
            return None
        if float(np.dot(line_dir, body_dir)) < 0.0:
            line_dir = -line_dir

        p_ref = np.array([float(x0), float(y0)], dtype=np.float32)
        line_normal = np.array([-line_dir[1], line_dir[0]], dtype=np.float32)
        residuals = np.abs((pts - p_ref) @ line_normal)
        inlier_thresh = max(
            float(cfg.SIDE_LINE_RESIDUAL_MAX_PX) * 1.35,
            float(np.percentile(residuals, 70.0))
        )
        inlier_mask = residuals <= inlier_thresh
        if np.count_nonzero(inlier_mask) >= cfg.BODY_SIDE_SUPPORT_MIN_POINTS and np.count_nonzero(inlier_mask) < len(pts):
            pts = pts[inlier_mask]
            vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
            line_dir = np.array([float(vx), float(vy)], dtype=np.float32)
            line_dir = self._canonicalize_2d_direction(line_dir)
            if line_dir is None:
                return None
            if float(np.dot(line_dir, body_dir)) < 0.0:
                line_dir = -line_dir
            p_ref = np.array([float(x0), float(y0)], dtype=np.float32)
            line_normal = np.array([-line_dir[1], line_dir[0]], dtype=np.float32)
            residuals = np.abs((pts - p_ref) @ line_normal)

        t_vals = (pts - p_ref) @ line_dir
        seg_p0 = p_ref + line_dir * float(np.min(t_vals))
        seg_p1 = p_ref + line_dir * float(np.max(t_vals))
        body_t = (pts - body_center) @ body_dir
        support_axis_span = float(np.max(body_t) - np.min(body_t)) if len(body_t) > 1 else 0.0
        signed_offset = float(np.mean((pts - body_center) @ body_normal))

        return {
            'label': label,
            'signed_offset': float(signed_offset),
            'residual_px': float(np.mean(residuals)) if len(residuals) > 0 else 1e6,
            'length_px': float(np.linalg.norm(seg_p1 - seg_p0)),
            'p0': seg_p0,
            'p1': seg_p1,
            'direction': line_dir,
            'support_count': int(len(pts)),
            'support_points': pts,
            'support_axis_span_px': float(support_axis_span),
            'span_ratio': float(support_axis_span / (long_span + 1e-6)),
            'body_alignment': abs(float(np.dot(line_dir, body_dir))),
            'accepted_for_oblique': False,
            'quality_reason': '',
        }

    def _line_quality_failure_reason(self, line):
        if line is None:
            return "line fit failed"
        if float(line.get('body_alignment', 0.0)) < cfg.BODY_SIDE_MIN_BODY_ALIGNMENT:
            return (
                f"{line.get('label', '?')} body alignment "
                f"{float(line.get('body_alignment', 0.0)):.2f}<{cfg.BODY_SIDE_MIN_BODY_ALIGNMENT:.2f}"
            )
        if float(line.get('span_ratio', 0.0)) < cfg.BODY_SIDE_MIN_SPAN_RATIO:
            return (
                f"{line.get('label', '?')} span "
                f"{float(line.get('span_ratio', 0.0)):.2f}<{cfg.BODY_SIDE_MIN_SPAN_RATIO:.2f}"
            )
        if float(line.get('residual_px', 1e6)) > cfg.SIDE_LINE_RESIDUAL_MAX_PX:
            return (
                f"{line.get('label', '?')} residual "
                f"{float(line.get('residual_px', 0.0)):.2f}>{cfg.SIDE_LINE_RESIDUAL_MAX_PX:.2f}"
            )
        return ""

    def _evaluate_side_line_pair(self, lines, body_geom):
        ordered_lines = sorted(lines, key=lambda item: item.get('signed_offset', 0.0))
        if len(ordered_lines) < 2 or body_geom is None:
            return ordered_lines, {
                'accepted': False,
                'reason': 'need two candidate side lines',
                'status': 'fit_incomplete',
                'pair_metrics': {},
            }

        line_reasons = []
        for line in ordered_lines:
            line['quality_reason'] = self._line_quality_failure_reason(line)
            line['accepted_for_oblique'] = (line['quality_reason'] == "")
            if line['quality_reason']:
                line_reasons.append(line['quality_reason'])

        short_span = max(1.0, float(body_geom['short_span']))
        parallel_score = abs(float(np.dot(ordered_lines[0]['direction'], ordered_lines[1]['direction'])))
        separation_px = abs(float(ordered_lines[1]['signed_offset'] - ordered_lines[0]['signed_offset']))
        separation_ratio = float(separation_px / (short_span + 1e-6))
        support_balance = float(
            min(ordered_lines[0]['support_count'], ordered_lines[1]['support_count'])
            / max(1, max(ordered_lines[0]['support_count'], ordered_lines[1]['support_count']))
        )
        min_span_ratio = float(min(ordered_lines[0]['span_ratio'], ordered_lines[1]['span_ratio']))
        mean_residual = float(np.mean([ordered_lines[0]['residual_px'], ordered_lines[1]['residual_px']]))
        mean_alignment = float(np.mean([ordered_lines[0]['body_alignment'], ordered_lines[1]['body_alignment']]))

        q_parallel = float(np.clip(
            (parallel_score - cfg.BODY_SIDE_PAIR_PARALLEL_MIN) / (1.0 - cfg.BODY_SIDE_PAIR_PARALLEL_MIN + 1e-6),
            0.0,
            1.0
        ))
        q_separation = float(np.clip(
            (separation_ratio - cfg.BODY_SIDE_MIN_SEPARATION_RATIO) / (1.0 - cfg.BODY_SIDE_MIN_SEPARATION_RATIO + 1e-6),
            0.0,
            1.0
        ))
        q_residual = float(np.clip(
            1.0 - mean_residual / (cfg.SIDE_LINE_RESIDUAL_MAX_PX + 1e-6),
            0.0,
            1.0
        ))
        pair_score = float(np.clip(
            0.25 * min_span_ratio
            + 0.20 * q_parallel
            + 0.20 * q_separation
            + 0.15 * q_residual
            + 0.10 * support_balance
            + 0.10 * mean_alignment,
            0.0,
            1.0
        ))
        pair_metrics = {
            'parallel_score': float(parallel_score),
            'separation_px': float(separation_px),
            'separation_ratio': float(separation_ratio),
            'support_balance': float(support_balance),
            'min_span_ratio': float(min_span_ratio),
            'mean_residual_px': float(mean_residual),
            'mean_alignment': float(mean_alignment),
            'pair_score': float(pair_score),
        }

        if line_reasons:
            return ordered_lines, {
                'accepted': False,
                'reason': '; '.join(line_reasons[:2]),
                'status': 'line_quality_failed',
                'pair_metrics': pair_metrics,
            }
        if parallel_score < cfg.BODY_SIDE_PAIR_PARALLEL_MIN:
            return ordered_lines, {
                'accepted': False,
                'reason': f"pair parallel {parallel_score:.2f}<{cfg.BODY_SIDE_PAIR_PARALLEL_MIN:.2f}",
                'status': 'pair_quality_failed',
                'pair_metrics': pair_metrics,
            }
        if separation_ratio < cfg.BODY_SIDE_MIN_SEPARATION_RATIO:
            return ordered_lines, {
                'accepted': False,
                'reason': f"pair separation {separation_ratio:.2f}<{cfg.BODY_SIDE_MIN_SEPARATION_RATIO:.2f}",
                'status': 'pair_quality_failed',
                'pair_metrics': pair_metrics,
            }
        if support_balance < cfg.BODY_SIDE_PAIR_SUPPORT_BALANCE_MIN:
            return ordered_lines, {
                'accepted': False,
                'reason': f"pair support balance {support_balance:.2f}<{cfg.BODY_SIDE_PAIR_SUPPORT_BALANCE_MIN:.2f}",
                'status': 'pair_quality_failed',
                'pair_metrics': pair_metrics,
            }
        if pair_score < cfg.BODY_SIDE_OBLIQUE_MIN_SCORE:
            return ordered_lines, {
                'accepted': False,
                'reason': f"pair score {pair_score:.2f}<{cfg.BODY_SIDE_OBLIQUE_MIN_SCORE:.2f}",
                'status': 'pair_quality_failed',
                'pair_metrics': pair_metrics,
            }

        for line in ordered_lines:
            line['accepted_for_oblique'] = True
            line['quality_reason'] = 'accepted'
        return ordered_lines, {
            'accepted': True,
            'reason': 'accepted',
            'status': 'accepted',
            'pair_metrics': pair_metrics,
        }

    def detect_body_side_lines(self, enhanced_gray, body_mask, body_geom=None):
        """以 body mask 约束搜索区域，再回到 RGB 中精修左右侧边。"""
        side_debug = {
            'status': 'not_attempted',
            'reason': 'side detection not run',
            'accepted': False,
            'body_geom': body_geom,
            'left_band_mask': None,
            'right_band_mask': None,
            'left_seed_points': np.empty((0, 2), dtype=np.float32),
            'right_seed_points': np.empty((0, 2), dtype=np.float32),
            'left_support_points': np.empty((0, 2), dtype=np.float32),
            'right_support_points': np.empty((0, 2), dtype=np.float32),
            'candidate_lines': [],
            'pair_metrics': {},
            'left_info': {},
            'right_info': {},
        }

        if body_mask is None:
            side_debug['status'] = 'missing_body_mask'
            side_debug['reason'] = 'body mask unavailable'
            return [], side_debug

        mask = (body_mask > 0).astype(np.uint8) * 255
        if np.count_nonzero(mask) < cfg.BODY_SIDE_MIN_POINTS:
            side_debug['status'] = 'body_mask_too_small'
            side_debug['reason'] = f"body mask pixels {int(np.count_nonzero(mask))}<{cfg.BODY_SIDE_MIN_POINTS}"
            return [], side_debug

        if body_geom is None:
            body_geom = self._estimate_body_geometry(mask, None)
            side_debug['body_geom'] = body_geom
        if body_geom is None:
            side_debug['status'] = 'body_axis_failed'
            side_debug['reason'] = 'body axis estimation failed'
            return [], side_debug

        band_info = self._build_side_search_bands(mask, body_geom)
        if band_info is None:
            side_debug['status'] = 'band_build_failed'
            side_debug['reason'] = 'side search band generation failed'
            return [], side_debug

        side_debug['left_band_mask'] = band_info['left_mask']
        side_debug['right_band_mask'] = band_info['right_mask']
        side_debug['left_seed_points'] = band_info['left_seed_points']
        side_debug['right_seed_points'] = band_info['right_seed_points']

        scharr_x = cv2.Scharr(enhanced_gray, cv2.CV_32F, 1, 0)
        scharr_y = cv2.Scharr(enhanced_gray, cv2.CV_32F, 0, 1)
        normal_dir = np.asarray(body_geom['normal_dir'], dtype=np.float32)
        grad_normal_abs = np.abs(scharr_x * normal_dir[0] + scharr_y * normal_dir[1])
        edge_gate = cv2.dilate(self.auto_canny(enhanced_gray), np.ones((3, 3), np.uint8), iterations=1)

        left_points, left_info = self._refine_side_support_points(
            band_info['left_mask'],
            edge_gate,
            grad_normal_abs,
            band_info['axis_proj'],
            band_info['normal_proj'],
            float(body_geom['side_offsets'][0]),
            'S0'
        )
        right_points, right_info = self._refine_side_support_points(
            band_info['right_mask'],
            edge_gate,
            grad_normal_abs,
            band_info['axis_proj'],
            band_info['normal_proj'],
            float(body_geom['side_offsets'][1]),
            'S1'
        )

        side_debug['left_support_points'] = left_points
        side_debug['right_support_points'] = right_points
        side_debug['left_info'] = left_info
        side_debug['right_info'] = right_info

        candidate_lines = []
        reasons = []
        if left_info.get('reason'):
            reasons.append(left_info['reason'])
        else:
            left_line = self._fit_refined_side_line(left_points, body_geom, 'S0')
            if left_line is None:
                reasons.append('S0 line fit failed')
            else:
                candidate_lines.append(left_line)

        if right_info.get('reason'):
            reasons.append(right_info['reason'])
        else:
            right_line = self._fit_refined_side_line(right_points, body_geom, 'S1')
            if right_line is None:
                reasons.append('S1 line fit failed')
            else:
                candidate_lines.append(right_line)

        side_debug['candidate_lines'] = candidate_lines
        if len(candidate_lines) < 2:
            side_debug['status'] = 'fit_incomplete'
            side_debug['reason'] = '; '.join(reasons[:2]) if reasons else 'need two fitted side lines'
            return [], side_debug

        candidate_lines, pair_eval = self._evaluate_side_line_pair(candidate_lines, body_geom)
        side_debug['candidate_lines'] = candidate_lines
        side_debug['pair_metrics'] = pair_eval.get('pair_metrics', {})
        side_debug['status'] = pair_eval.get('status', 'pair_quality_failed')
        side_debug['reason'] = pair_eval.get('reason', 'pair quality failed')
        side_debug['accepted'] = bool(pair_eval.get('accepted', False))

        if not side_debug['accepted']:
            return [], side_debug

        return candidate_lines[:2], side_debug

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

    def score_candidates_rgb_only(self, pose_candidates, body_geom=None):
        """第二阶段第一步：仅用当前 RGB 几何和 body 主轴做双候选消歧。"""
        if not pose_candidates:
            return [], None

        scored_candidates = []

        for candidate in pose_candidates:
            cand = dict(candidate)
            if body_geom is None:
                cand['body_axis_score'] = 0.5
                cand['body_center_score'] = 0.5
                cand['centerline_score'] = 0.5
                cand['rgb_only_score'] = 0.5
                cand['rgb_only_reason'] = 'no body geometry, keep neutral score'
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
            half_span = max(6.0, 0.5 * float(body_geom.get('short_span', min(body_geom.get('bw', 0.0), body_geom.get('bh', 0.0)))))
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

    def check_body_rim_consistency(self, body_geom, ellipse_2d, normal):
        """Body-Rim 联合约束一致性分数（0~1）。"""
        if body_geom is None or ellipse_2d is None or normal is None:
            return 1.0

        try:
            body_dir = np.asarray(body_geom.get('body_dir', np.array([1.0, 0.0], dtype=np.float32)), dtype=np.float32)
            body_dir_norm = float(np.linalg.norm(body_dir))
            if body_dir_norm < 1e-6:
                return 0.5
            body_dir = body_dir / body_dir_norm

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
        selected_candidate,
        side_debug=None
    ):
        debug_vis = roi_img.copy()

        if cfg.ROI_DEBUG_SHOW_EXTRACTION_LAYERS:
            band_overlay = debug_vis.copy()
            band_overlay[boundary_band > 0] = (0, 180, 0)
            debug_vis = cv2.addWeighted(debug_vis, 0.82, band_overlay, 0.18, 0.0)
            debug_vis[edges_filtered > 0] = (255, 120, 0)

            for seg in arc_segments:
                pts = seg['points'].astype(np.int32).reshape(-1, 1, 2)
                cv2.polylines(debug_vis, [pts], False, (0, 220, 255), 1)

            for line in line_segments:
                p0 = tuple(line['points'][0].astype(int))
                p1 = tuple(line['points'][1].astype(int))
                cv2.line(debug_vis, p0, p1, (255, 255, 0), 1)

        alt_candidates = candidates[: max(0, int(cfg.ROI_DEBUG_ALT_CANDIDATES))]
        for idx, candidate in enumerate(alt_candidates):
            color = (90, 90, 90) if idx > 0 else (0, 215, 255)
            thickness = 1 if idx > 0 else 2
            cv2.ellipse(debug_vis, candidate['ellipse'], color, thickness)

        if selected_candidate is not None:
            cv2.ellipse(debug_vis, selected_candidate['ellipse'], (0, 0, 255), 2)
            cx, cy = selected_candidate['ellipse'][0]
            cv2.drawMarker(
                debug_vis,
                (int(round(cx)), int(round(cy))),
                (0, 0, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=10,
                thickness=1
            )

        if cfg.ROI_DEBUG_SHOW_SIDE_DIAGNOSTICS and side_debug:
            band_overlay = debug_vis.copy()
            left_band = side_debug.get('left_band_mask', None)
            right_band = side_debug.get('right_band_mask', None)
            if left_band is not None:
                band_overlay[left_band > 0] = (255, 90, 220)
            if right_band is not None:
                band_overlay[right_band > 0] = (80, 245, 220)
            debug_vis = cv2.addWeighted(debug_vis, 0.84, band_overlay, 0.16, 0.0)

            body_geom = side_debug.get('body_geom', None)
            if body_geom is not None:
                center_uv = np.asarray(body_geom['center_uv'], dtype=np.float32)
                body_dir = np.asarray(body_geom['body_dir'], dtype=np.float32)
                axis_half = 0.5 * float(body_geom.get('long_span', 0.0))
                axis_p0 = center_uv - body_dir * axis_half
                axis_p1 = center_uv + body_dir * axis_half
                cv2.line(
                    debug_vis,
                    tuple(np.round(axis_p0).astype(int)),
                    tuple(np.round(axis_p1).astype(int)),
                    (60, 255, 100),
                    1,
                )
                cv2.drawMarker(
                    debug_vis,
                    tuple(np.round(center_uv).astype(int)),
                    (60, 255, 100),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=8,
                    thickness=1,
                )

            def draw_points(points_xy, color, radius, stride_target):
                pts = np.asarray(points_xy, dtype=np.float32).reshape(-1, 2)
                if len(pts) == 0:
                    return
                stride = max(1, len(pts) // max(1, stride_target))
                for pt in pts[::stride]:
                    cv2.circle(
                        debug_vis,
                        tuple(np.round(pt).astype(int)),
                        radius,
                        color,
                        -1,
                        cv2.LINE_AA
                    )

            draw_points(side_debug.get('left_seed_points', np.empty((0, 2))), (200, 110, 255), 1, 80)
            draw_points(side_debug.get('right_seed_points', np.empty((0, 2))), (110, 255, 235), 1, 80)
            draw_points(side_debug.get('left_support_points', np.empty((0, 2))), (255, 60, 210), 2, 60)
            draw_points(side_debug.get('right_support_points', np.empty((0, 2))), (70, 255, 220), 2, 60)

            for idx, line in enumerate(side_debug.get('candidate_lines', [])[:2]):
                accepted = bool(line.get('accepted_for_oblique', False))
                color = (255, 70, 220) if idx == 0 else (70, 255, 220)
                if not accepted:
                    color = tuple(int(c * 0.65) for c in color)
                p0 = tuple(np.round(line['p0']).astype(int))
                p1 = tuple(np.round(line['p1']).astype(int))
                cv2.line(debug_vis, p0, p1, color, 2 if accepted else 1)
                mid = (
                    int(round((line['p0'][0] + line['p1'][0]) * 0.5)),
                    int(round((line['p0'][1] + line['p1'][1]) * 0.5))
                )
                label = str(line.get('label', f'S{idx}'))
                if not accepted:
                    label += "x"
                cv2.putText(
                    debug_vis,
                    label,
                    (mid[0] + 4, mid[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    color,
                    1,
                    cv2.LINE_AA
                )

        return debug_vis

    def _draw_debug_info_card(self, image, lines, origin=(8, 8), accent_color=(180, 255, 180)):
        if image is None or not lines:
            return image

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.44
        thickness = 1
        line_height = 17
        padding = 8

        max_width = 0
        for line in lines:
            (w, _), _ = cv2.getTextSize(line, font, font_scale, thickness)
            max_width = max(max_width, w)

        x, y = origin
        box_w = max_width + padding * 2
        box_h = len(lines) * line_height + padding * 2
        x2 = min(image.shape[1] - 1, x + box_w)
        y2 = min(image.shape[0] - 1, y + box_h)

        overlay = image.copy()
        cv2.rectangle(overlay, (x, y), (x2, y2), (24, 24, 24), -1)
        cv2.rectangle(overlay, (x, y), (x2, y2), accent_color, 1)
        cv2.addWeighted(overlay, 0.62, image, 0.38, 0.0, image)

        text_y = y + padding + 12
        for idx, line in enumerate(lines):
            color = accent_color if idx == 0 else (235, 235, 235)
            cv2.putText(
                image,
                line,
                (x + padding, text_y),
                font,
                font_scale,
                color,
                thickness,
                cv2.LINE_AA
            )
            text_y += line_height
        return image

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

        pose_candidates = pose_result.get('pose_candidates_3d', [])
        selected_id = int(pose_result.get('selected_pose_candidate_id', -1))
        selected_pose = next(
            (cand for cand in pose_candidates if int(cand.get('candidate_id', -99)) == selected_id),
            None
        )

        lines = [
            f"POSE {pose_result.get('observability_state', '?').upper()}  P{selected_id}  ar={float(pose_result.get('axis_ratio', 0.0)):.2f}"
        ]
        lines.append(
            f"Q={float(pose_result.get('quality_score', 0.0)):.2f}  "
            f"Fit={float(pose_result.get('ellipse_fit_score', 0.0)):.2f}  "
            f"Cons={float(pose_result.get('consistency_score', 0.0)):.2f}"
        )

        score_tokens = []
        for cand in pose_candidates[:2]:
            score_tokens.append(
                f"P{int(cand.get('candidate_id', -1))}:{float(cand.get('rgb_only_score', 0.0)):.2f}"
            )
        if score_tokens:
            lines.append("  ".join(score_tokens))

        side_lines = pose_result.get('side_lines_2d', [])
        side_status = str(pose_result.get('side_line_status', ''))
        side_ready = bool(pose_result.get('side_oblique_ready', False))
        side_pair = pose_result.get('side_line_pair_metrics', {})
        if side_status:
            lines.append(f"SIDE {'READY' if side_ready else 'HOLD'} {side_status}")
        if side_pair:
            lines.append(
                f"par={float(side_pair.get('parallel_score', 0.0)):.2f}  "
                f"sep={float(side_pair.get('separation_ratio', 0.0)):.2f}  "
                f"bal={float(side_pair.get('support_balance', 0.0)):.2f}  "
                f"pair={float(side_pair.get('pair_score', 0.0)):.2f}"
            )
        if len(side_lines) >= 2:
            lines.append(
                f"S0:{float(side_lines[0].get('span_ratio', 0.0)):.2f}/{float(side_lines[0].get('residual_px', 0.0)):.2f}  "
                f"S1:{float(side_lines[1].get('span_ratio', 0.0)):.2f}/{float(side_lines[1].get('residual_px', 0.0)):.2f}"
            )

        if selected_pose is not None:
            lines.append(
                f"ctr={float(selected_pose.get('body_center_score', 0.0)):.2f}  "
                f"mid={float(selected_pose.get('centerline_score', 0.0)):.2f}  "
                f"ax={float(selected_pose.get('body_axis_score', 0.0)):.2f}"
            )
        elif pose_result.get('selected_pose_reason', ''):
            lines.append(str(pose_result.get('selected_pose_reason', ''))[:64])

        if not side_ready and pose_result.get('side_line_reject_reason', ''):
            lines.append(str(pose_result.get('side_line_reject_reason', ''))[:68])

        return self._draw_debug_info_card(
            debug_vis_img,
            lines,
            origin=(8, 8),
            accent_color=(170, 255, 170)
        )

    def process_detection(
        self,
        image,
        precise_rim_mask,
        crop_offset,
        body_bbox=None,
        body_mask=None,
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
        roi_body_mask = None if body_mask is None else body_mask[y1:y2, x1:x2]
        local_body_bbox = None
        if body_bbox is not None:
            local_body_bbox = (
                float(body_bbox[0]) - float(x1),
                float(body_bbox[1]) - float(y1),
                float(body_bbox[2]),
                float(body_bbox[3]),
            )
        if roi_img.size == 0:
            return None, "Empty ROI", [], None, (x1, y1)

        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.bilateralFilter(gray, 5, 50, 50)
        enhanced = self.clahe.apply(gray_blur)
        body_geom_local = self._estimate_body_geometry(roi_body_mask, body_bbox=local_body_bbox)
        body_geom_global = self._offset_body_geometry(body_geom_local, (x1, y1))

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

        side_lines, side_debug = self.detect_body_side_lines(
            enhanced,
            roi_body_mask,
            body_geom=body_geom_local
        )
        debug_vis_img = self._build_debug_visualization(
            roi_img,
            boundary_band,
            edges_filtered,
            arc_segments,
            line_segments,
            candidates,
            selected_candidate,
            side_debug=side_debug
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
            body_geom=body_geom_global
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
            body_geom_local,
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
        pose_result['side_line_status'] = str(side_debug.get('status', 'not_attempted'))
        pose_result['side_line_reject_reason'] = str(side_debug.get('reason', ''))
        pose_result['side_oblique_ready'] = bool(side_debug.get('accepted', False))
        pose_result['side_line_candidate_count'] = int(len(side_debug.get('candidate_lines', [])))
        pose_result['body_axis_source'] = str(body_geom_local.get('source', 'none')) if body_geom_local is not None else 'none'
        pair_metrics = side_debug.get('pair_metrics', {})
        pose_result['side_line_pair_metrics'] = {
            key: float(value)
            for key, value in pair_metrics.items()
        }
        pose_result['side_line_count'] = len(side_lines)
        pose_result['side_lines_2d'] = [
            {
                'label': str(line.get('label', f'S{idx}')),
                'p0_uv': [float(line['p0'][0] + x1), float(line['p0'][1] + y1)],
                'p1_uv': [float(line['p1'][0] + x1), float(line['p1'][1] + y1)],
                'residual_px': float(line.get('residual_px', 0.0)),
                'span_ratio': float(line.get('span_ratio', 0.0)),
                'body_alignment': float(line.get('body_alignment', 0.0)),
            }
            for idx, line in enumerate(side_lines[:2])
        ]
        if pose_result['side_oblique_ready'] and len(side_lines) >= 2 and not pose_result.get('near_circle_degenerate', False):
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
