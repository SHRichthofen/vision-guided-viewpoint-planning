#!/usr/bin/env python3
"""
圆柱检测ROS2节点
功能：
  1. 订阅RealSense相机RGB流
  2. 使用YOLO两阶段检测（圆柱+孔口）
  3. 计算圆柱3D位姿
  4. 发布圆柱口中心与轴向语义

发布话题：
  /vision/cylinder_pose (geometry_msgs/PoseStamped): 相机坐标系中的圆柱口中心（orientation 为占位值）
  /vision/selected_cylinder_semantics (std_msgs/String): 相机坐标系中的圆柱口中心与轴向语义
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import Int32, String
import numpy as np
import cv2
import os
import math
import json
from ament_index_python.packages import get_package_share_directory

# 有条件地导入外部库
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

from .pose_estimator import RGBCylinderPoseEstimator
from .utils import get_centered_crop_coords
from . import config as cfg


class CylinderDetectionNode(Node):
    def __init__(self):
        super().__init__('cylinder_detection')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("圆柱检测节点启动 (Cylinder Detection Node)")
        self.get_logger().info("=" * 60)
        
        # ==================== 检查依赖 ====================
        if not YOLO_AVAILABLE:
            self.get_logger().error("✗ 缺少 ultralytics 模块，请运行: pip install ultralytics")
            raise ImportError("ultralytics not found")
        
        if not REALSENSE_AVAILABLE:
            self.get_logger().warn("⚠ 缺少 pyrealsense2 模块，仅限仿真测试。请运行: pip install pyrealsense2")
        
        # ==================== 参数声明 ====================
        self.declare_parameter('model_dir', '')
        self.declare_parameter('enable_debug', False)
        self.declare_parameter('publish_interval_ms', 100)
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('require_confirm_before_publish', True)
        self.declare_parameter('scan_distance', 0.05)
        self.declare_parameter('look_inward', True)
        self.declare_parameter('publish_once_per_selection', cfg.PUBLISH_ONCE_PER_SELECTION)
        self.declare_parameter('selected_semantics_topic', cfg.SELECTED_SEMANTICS_TOPIC)
        self.declare_parameter('enable_depth_prior', cfg.ENABLE_DEPTH_PRIOR)
        self.declare_parameter('depth_min_m', cfg.DEPTH_MIN_M)
        self.declare_parameter('depth_max_m', cfg.DEPTH_MAX_M)
        self.declare_parameter('depth_valid_ratio_min', cfg.DEPTH_VALID_RATIO_MIN)
        self.declare_parameter('depth_iqr_max_m', cfg.DEPTH_IQR_MAX_M)
        
        # ==================== 获取参数 ====================
        model_dir = self.get_parameter('model_dir').value
        self.debug = self.get_parameter('enable_debug').value
        publish_interval = self.get_parameter('publish_interval_ms').value
        self.enable_visualization = self.get_parameter('enable_visualization').value
        self.require_confirm = self.get_parameter('require_confirm_before_publish').value
        self.scan_distance = self.get_parameter('scan_distance').value
        self.look_inward = self.get_parameter('look_inward').value
        self.publish_once_per_selection = bool(self.get_parameter('publish_once_per_selection').value)
        self.selected_semantics_topic = self.get_parameter('selected_semantics_topic').value
        self.enable_depth_prior = bool(self.get_parameter('enable_depth_prior').value)
        self.depth_min_m = float(self.get_parameter('depth_min_m').value)
        self.depth_max_m = float(self.get_parameter('depth_max_m').value)
        self.depth_valid_ratio_min = float(self.get_parameter('depth_valid_ratio_min').value)
        self.depth_iqr_max_m = float(self.get_parameter('depth_iqr_max_m').value)
        
        # ==================== 初始化硬件和模型 ====================
        try:
            if not model_dir:
                # 使用包内的模型
                pkg_dir = get_package_share_directory('vision_detection')
                model_dir = os.path.join(pkg_dir, 'models')
            
            self.get_logger().info(f"加载模型从: {model_dir}")
            model_body_path = os.path.join(model_dir, cfg.MODEL_BODY_FILENAME)
            model_rim_path = os.path.join(model_dir, cfg.MODEL_RIM_FILENAME)
            
            self.model_body = YOLO(model_body_path)
            self.model_rim = YOLO(model_rim_path)
            self.get_logger().info("✓ 模型加载成功")
        except Exception as e:
            self.get_logger().error(f"✗ 模型加载失败: {e}")
            raise
        
        try:
            self.get_logger().info("启动RealSense相机...")
            self.pipeline = rs.pipeline()
            rs_cfg = rs.config()
            rs_cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            rs_cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            profile = self.pipeline.start(rs_cfg)
            self.depth_align = rs.align(rs.stream.color)
            self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            
            # 设置抗频闪（非致命）
            try:
                sensor = profile.get_device().query_sensors()[1]
                sensor.set_option(rs.option.power_line_frequency, 1)
            except Exception as opt_err:
                self.get_logger().warn(f"设置抗频闪失败，继续运行: {opt_err}")
            
            self.intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.get_logger().info("✓ RealSense相机初始化成功")
        except Exception as e:
            self.get_logger().error(f"✗ 相机初始化失败: {e}")
            raise
        
        # ==================== 初始化处理器 ====================
        self.pose_estimator = RGBCylinderPoseEstimator(self.intr)
        
        # ==================== 发布器 ====================
        # 发布所有检测到的圆柱（PoseArray）
        self.cylinders_pub = self.create_publisher(
            PoseArray,
            '/vision/cylinders',
            10
        )
        
        # 兼容旧接口：发布当前选中目标的中心点，orientation 仅为占位值
        self.target_info_pub = self.create_publisher(
            PoseStamped,
            '/vision/cylinder_pose',
            10
        )

        self.selected_semantics_pub = self.create_publisher(
            String,
            self.selected_semantics_topic,
            10
        )
        
        # 订阅目标选择
        self.target_id_sub = self.create_subscription(
            Int32,
            '/target_id',
            self.on_target_selected,
            10
        )

        self.target_id_pub = self.create_publisher(
            Int32,
            '/target_id',
            10
        )
        
        # 维护状态
        self.selected_target_id = None
        self.latest_cylinders = {}  # track_id -> pose_result
        self.ui_candidate_id = None
        self.target_pose_sent = False
        
        # ==================== 定时器 ====================
        self.create_timer(
            publish_interval / 1000.0,
            self.detection_loop
        )
        
        self.get_logger().info("✓ 圆柱检测节点初始化成功")
        self.get_logger().info(f"  相机内参: fx={self.intr.fx}, fy={self.intr.fy}")
        self.get_logger().info(f"  发布频率: {1000/publish_interval:.1f} Hz\n")

        if self.enable_visualization:
            self.window_name = "Cylinder Detection - Live View"
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            self.get_logger().info("可视化选目标: [A/D]切换  [ENTER]确认  [X]取消")
    
    def detection_loop(self):
        """主检测循环"""
        try:
            # 获取图像
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            aligned_frames = self.depth_align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame:
                return
            
            image = np.asanyarray(color_frame.get_data())
            depth_image_m = None
            if self.enable_depth_prior and depth_frame:
                depth_image_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * float(self.depth_scale)

            vis_image = image.copy()
            debug_canvas = np.zeros_like(image)
            self.latest_cylinders = {}
            detected_tubes = []
            
            # ==================== Stage 1: 圆柱检测 ====================
            results_body = self.model_body.track(
                image,
                persist=True,
                verbose=False,
                tracker="bytetrack.yaml"
            )
            detected_tubes = self.parse_body_detections(results_body)

            current_ids = [t['track_id'] for t in detected_tubes]
            if len(current_ids) > 0:
                detected_tubes.sort(key=lambda x: x['center_x'])
                if self.ui_candidate_id not in current_ids:
                    self.ui_candidate_id = detected_tubes[0]['track_id']
                if self.selected_target_id is not None and self.selected_target_id not in current_ids:
                    self.selected_target_id = None
                    self.target_pose_sent = False
            else:
                self.ui_candidate_id = None

            """ if self.debug:
                self.get_logger().info(f"检测到 {len(detected_tubes)} 个圆柱") """

            # ==================== Stage 2: Rim流水线（对每个圆柱） ====================
            for tube in detected_tubes:
                track_id = tube['track_id']

                depth_prior_m = None
                depth_prior_confidence = 0.0
                depth_prior_metrics = None
                if self.enable_depth_prior and depth_image_m is not None:
                    depth_prior_m, depth_prior_confidence, depth_prior_metrics = self.compute_depth_prior_from_body_mask(
                        depth_image_m,
                        tube.get('raw_body_mask'),
                        tube.get('bbox_xywh'),
                    )

                raw_pose, vis_rim_mask, debug_roi, crop_xy = self.run_rim_pipeline(
                    image,
                    tube,
                    depth_prior_m=depth_prior_m,
                    depth_prior_confidence=depth_prior_confidence
                )
                is_candidate = (self.ui_candidate_id is not None and tube['track_id'] == self.ui_candidate_id)

                # body mask（仅用于可视化）
                vis_body_mask = None
                if tube.get('raw_body_mask') is not None:
                    m = cv2.resize(tube['raw_body_mask'], (image.shape[1], image.shape[0]))
                    vis_body_mask = (m > 0.5).astype(np.uint8) * 255

                # 调试ROI拼接到右侧画布
                if debug_roi is not None:
                    dx, dy = crop_xy
                    h, w = debug_roi.shape[:2]
                    ey, ex = min(dy + h, image.shape[0]), min(dx + w, image.shape[1])
                    debug_canvas[dy:ey, dx:ex] = debug_roi[:ey - dy, :ex - dx]

                # 当前工作流不做失锁续航：rim失败直接跳过该帧
                if raw_pose is None:
                    continue

                raw_pose['track_id'] = int(track_id)
                raw_pose['body_bbox'] = tuple(float(v) for v in tube.get('bbox_xywh', [0, 0, 0, 0]))
                if depth_prior_metrics is not None:
                    raw_pose['depth_prior_metrics'] = depth_prior_metrics
                self.latest_cylinders[track_id] = raw_pose

                # 可视化简化：仅在“当前选定框（candidate）”显示姿态和目标位姿
                if is_candidate:
                    self.draw_segmentation_masks(vis_image, vis_body_mask, vis_rim_mask, alpha=0.4)

                if is_candidate:
                    self.draw_pose_info(
                        vis_image,
                        raw_pose,
                        is_target=(self.selected_target_id is not None and tube['track_id'] == self.selected_target_id)
                    )
                    self.draw_target_pose_info(vis_image, raw_pose)

                # 调试日志改为仅在“目标确认并发布”时输出（见 publish_selected_target）

            """ if self.debug and len(detected_tubes) == 0:
                self.get_logger().debug("未检测到圆柱") """
            
            # ==================== 发布圆柱数组 ====================
            if self.latest_cylinders:
                self.publish_cylinders_array()
            
            # ==================== 本地图像可视化（源码风格） ====================
            if self.enable_visualization:
                for tube in detected_tubes:
                    is_target = (self.selected_target_id is not None and tube['track_id'] == self.selected_target_id)
                    is_candidate = (self.ui_candidate_id is not None and tube['track_id'] == self.ui_candidate_id)
                    self.draw_detection_box(vis_image, tube, is_target=is_target, is_candidate=is_candidate)

                cv2.putText(vis_image, f"Detected: {len(detected_tubes)}", (15, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                if self.ui_candidate_id is not None:
                    cv2.putText(vis_image, f"Candidate ID: {self.ui_candidate_id}", (15, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
                if self.selected_target_id is not None:
                    cv2.putText(vis_image, f"Selected ID: {self.selected_target_id}", (15, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.putText(vis_image, "[A/D] Switch  [ENTER] Confirm  [X] Clear", (15, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

                combined_view = np.hstack((vis_image, debug_canvas))
                cv2.imshow(self.window_name, combined_view)
                key = cv2.waitKey(1) & 0xFF
                self.handle_ui_key(key, detected_tubes)
            
            # ==================== 如果有选中目标，发布单个目标位姿 ====================
            can_publish = (self.selected_target_id is not None and self.selected_target_id in self.latest_cylinders)
            if not self.require_confirm and self.ui_candidate_id is not None and self.ui_candidate_id in self.latest_cylinders:
                if self.selected_target_id != self.ui_candidate_id:
                    self.target_pose_sent = False
                self.selected_target_id = self.ui_candidate_id
                can_publish = True

            if can_publish:
                self.publish_selected_target()
        
        except Exception as e:
            self.get_logger().error(f"检测循环错误: {e}")
    
    def publish_cylinders_array(self):
        """发布所有检测到的圆柱中心为 PoseArray，orientation 仅为占位值。"""
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "camera_color_optical_frame"
        
        for track_id in sorted(self.latest_cylinders.keys()):
            pose_result = self.latest_cylinders[track_id]
            pose = Pose()
            pose.position.x = float(pose_result['center_3d'][0])
            pose.position.y = float(pose_result['center_3d'][1])
            pose.position.z = float(pose_result['center_3d'][2])
            self._set_placeholder_orientation(pose)
            pose_array.poses.append(pose)
        
        self.cylinders_pub.publish(pose_array)
    
    def publish_selected_target(self):
        """发布选中目标的中心点与语义轴向。"""
        if self.publish_once_per_selection and self.target_pose_sent:
            return

        pose_result = self.latest_cylinders[self.selected_target_id]
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_color_optical_frame"
        pose_msg.pose.position.x = float(pose_result['center_3d'][0])
        pose_msg.pose.position.y = float(pose_result['center_3d'][1])
        pose_msg.pose.position.z = float(pose_result['center_3d'][2])
        self._set_placeholder_orientation(pose_msg.pose)

        if self.debug:
            quality_score = float(pose_result.get('quality_score', 1.0))
            qc = pose_result.get('quality_components', {})
            z_ellipse = float(pose_result.get('z_ellipse', pose_result['center_3d'][2]))
            z_prior = pose_result.get('z_prior', None)
            z_fused = float(pose_result.get('z_fused', pose_result['center_3d'][2]))
            z_w = float(pose_result.get('z_fusion_weight', 0.0))
            dp = pose_result.get('depth_prior_metrics', {})
            z_prior_text = 'None' if z_prior is None else f"{float(z_prior):.4f}"
            self.get_logger().info(
                f"[SELECTED] 圆柱 {self.selected_target_id}: "
                f"位置=[{pose_result['center_3d'][0]:.4f}, {pose_result['center_3d'][1]:.4f}, {pose_result['center_3d'][2]:.4f}], "
                f"法向=[{pose_result['normal'][0]:.4f}, {pose_result['normal'][1]:.4f}, {pose_result['normal'][2]:.4f}], "
                f"quality={quality_score:.3f} "
                f"(area={qc.get('q_area', 0.0):.2f}, cov={qc.get('q_coverage', 0.0):.2f}, "
                f"res={qc.get('q_residual', 0.0):.2f}, axis={qc.get('q_axis', 0.0):.2f}), "
                f"z=[ellipse:{z_ellipse:.4f}, prior:{z_prior_text}, fused:{z_fused:.4f}, w:{z_w:.2f}], "
                f"depth(valid={float(dp.get('valid_ratio', 0.0)):.2f}, iqr={float(dp.get('iqr_m', 0.0)):.4f})"
            )
        
        self.target_info_pub.publish(pose_msg)
        semantics_msg = String()
        semantics_msg.data = json.dumps(self._build_selected_semantics_payload(self.selected_target_id, pose_result), ensure_ascii=False)
        self.selected_semantics_pub.publish(semantics_msg)
        self.target_pose_sent = True

    def _set_placeholder_orientation(self, pose):
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

    def compute_depth_prior_from_body_mask(self, depth_image_m, raw_body_mask, bbox_xywh):
        """基于Stage1 body mask统计深度先验（鲁棒中位数+IQR）。"""
        if depth_image_m is None or raw_body_mask is None or bbox_xywh is None:
            return None, 0.0, None

        h, w = depth_image_m.shape[:2]
        resized = cv2.resize(raw_body_mask, (w, h), interpolation=cv2.INTER_LINEAR)
        mask = (resized > 0.5).astype(np.uint8)
        if np.count_nonzero(mask) < 50:
            return None, 0.0, None

        cx, cy, bw, bh = bbox_xywh
        x1 = max(0, int(cx - bw / 2))
        y1 = max(0, int(cy - bh / 2))
        x2 = min(w, int(cx + bw / 2))
        y2 = min(h, int(cy + bh / 2))
        roi_mask = np.zeros_like(mask)
        roi_mask[y1:y2, x1:x2] = 1
        valid_mask = (mask > 0) & (roi_mask > 0)

        dvals = depth_image_m[valid_mask]
        if dvals.size == 0:
            return None, 0.0, None

        finite_mask = np.isfinite(dvals)
        range_mask = (dvals >= self.depth_min_m) & (dvals <= self.depth_max_m)
        dvals = dvals[finite_mask & range_mask]
        mask_count = max(1, int(np.count_nonzero(valid_mask)))
        valid_ratio = float(dvals.size / mask_count)
        if dvals.size < 20 or valid_ratio < self.depth_valid_ratio_min:
            return None, 0.0, {
                'valid_ratio': valid_ratio,
                'iqr_m': float('inf'),
                'count': int(dvals.size),
            }

        p_low = np.percentile(dvals, cfg.DEPTH_PERCENTILE_LOW)
        p_high = np.percentile(dvals, cfg.DEPTH_PERCENTILE_HIGH)
        trimmed = dvals[(dvals >= p_low) & (dvals <= p_high)]
        if trimmed.size < 10:
            trimmed = dvals

        d_med = float(np.median(trimmed))
        q1 = float(np.percentile(trimmed, 25))
        q3 = float(np.percentile(trimmed, 75))
        iqr = q3 - q1
        d_std = float(np.std(trimmed))

        iqr_term = float(np.clip(1.0 - (iqr / max(1e-6, self.depth_iqr_max_m)), 0.0, 1.0))
        conf = float(np.clip(0.6 * valid_ratio + 0.4 * iqr_term, 0.0, 1.0))

        metrics = {
            'depth_prior_m': float(d_med),
            'valid_ratio': float(valid_ratio),
            'iqr_m': float(iqr),
            'std_m': float(d_std),
            'count': int(trimmed.size),
            'confidence': float(conf),
        }
        return float(d_med), float(conf), metrics

    def parse_body_detections(self, results):
        """按源码逻辑解析 Stage1 body 结果"""
        detected_tubes = []
        if not results:
            return detected_tubes

        if results[0].boxes is not None and results[0].boxes.id is not None:
            boxes_xywh = results[0].boxes.xywh.cpu().numpy()
            track_ids = results[0].boxes.id.int().cpu().tolist()

            raw_masks = None
            if results[0].masks is not None:
                raw_masks = results[0].masks.data.cpu().numpy()

            for i, track_id in enumerate(track_ids):
                bbox = boxes_xywh[i]
                detected_tubes.append({
                    'track_id': int(track_id),
                    'bbox_xywh': bbox,
                    'center_x': bbox[0],
                    'center_y': bbox[1],
                    'raw_body_mask': raw_masks[i] if raw_masks is not None and i < len(raw_masks) else None
                })
        return detected_tubes

    def _build_selected_semantics_payload(self, track_id, pose):
        payload = {
            'measurement_kind': 'center_axis',
            'track_id': int(track_id),
            'frame_id': 'camera_color_optical_frame',
            'top_center_cam': [float(x) for x in pose['center_3d']],
            'axis_cam': [float(x) for x in pose['normal']],
            'axis_ratio': float(pose.get('axis_ratio', 0.0)),
            'quality_score': float(pose.get('quality_score', 0.0)),
            'near_circle_degenerate': bool(pose.get('near_circle_degenerate', False)),
            'z_ellipse': float(pose.get('z_ellipse', pose['center_3d'][2])),
            'z_fused': float(pose.get('z_fused', pose['center_3d'][2])),
            'z_prior': None if pose.get('z_prior', None) is None else float(pose.get('z_prior')),
            'z_fusion_weight': float(pose.get('z_fusion_weight', 0.0)),
            'depth_prior_confidence': float(pose.get('depth_prior_confidence', 0.0)),
            'stamp_sec': float(self.get_clock().now().nanoseconds) * 1e-9,
        }
        if 'observability_state' in pose:
            payload['observability_state'] = str(pose['observability_state'])
        if 'rim_pipeline_stage' in pose:
            payload['rim_pipeline_stage'] = str(pose['rim_pipeline_stage'])
        if 'selected_candidate_reason' in pose:
            payload['selected_candidate_reason'] = str(pose['selected_candidate_reason'])
        if 'selected_pose_candidate_id' in pose:
            payload['selected_pose_candidate_id'] = int(pose['selected_pose_candidate_id'])
        if 'selected_pose_reason' in pose:
            payload['selected_pose_reason'] = str(pose['selected_pose_reason'])
        if 'rim_candidates' in pose:
            payload['rim_candidates'] = pose['rim_candidates']
        if 'pose_candidates_3d' in pose:
            payload['pose_candidates_3d'] = pose['pose_candidates_3d']
        if 'depth_prior_ignored' in pose:
            payload['depth_prior_ignored'] = bool(pose['depth_prior_ignored'])
        if 'side_line_count' in pose:
            payload['side_line_count'] = int(pose['side_line_count'])
        if 'side_lines_2d' in pose:
            payload['side_lines_2d'] = pose['side_lines_2d']
        if 'side_line_status' in pose:
            payload['side_line_status'] = str(pose['side_line_status'])
        if 'side_line_reject_reason' in pose:
            payload['side_line_reject_reason'] = str(pose['side_line_reject_reason'])
        if 'side_line_pair_metrics' in pose:
            payload['side_line_pair_metrics'] = {
                k: float(v) for k, v in pose['side_line_pair_metrics'].items()
            }
        if 'side_disambiguation' in pose:
            payload['side_disambiguation'] = pose['side_disambiguation']
        if 'ellipse_2d' in pose:
            payload['ellipse_2d'] = {
                'center_uv': [float(pose['ellipse_2d'][0][0]), float(pose['ellipse_2d'][0][1])],
                'axes_px': [float(pose['ellipse_2d'][1][0]), float(pose['ellipse_2d'][1][1])],
                'angle_deg': float(pose['ellipse_2d'][2]),
            }
        if 'body_bbox' in pose:
            payload['body_bbox_xywh'] = [float(v) for v in pose['body_bbox']]
        if 'quality_components' in pose:
            payload['quality_components'] = {
                k: float(v) for k, v in pose['quality_components'].items()
            }
        if 'depth_prior_metrics' in pose:
            payload['depth_prior_metrics'] = {
                k: float(v) if isinstance(v, (float, int, np.floating, np.integer)) else v
                for k, v in pose['depth_prior_metrics'].items()
            }
        return payload

    def _select_best_rim_mask(self, masks_data, crop_shape):
        crop_h, crop_w = crop_shape[:2]
        crop_center = np.array([crop_w * 0.5, crop_h * 0.5], dtype=np.float32)
        best_mask = None
        best_score = -1e9

        for m in masks_data:
            m_resized = cv2.resize(m, (crop_w, crop_h))
            m_u8 = (m_resized > 0.5).astype(np.uint8) * 255
            area_px = float(np.count_nonzero(m_u8))
            if area_px <= 0.0:
                continue

            ys, xs = np.where(m_u8 > 0)
            centroid = np.array([np.mean(xs), np.mean(ys)], dtype=np.float32)
            center_dist = float(np.linalg.norm(centroid - crop_center))
            norm_center_dist = center_dist / max(1.0, np.linalg.norm(crop_center))
            score = area_px * (1.0 - cfg.RIM_SELECTION_CENTER_BIAS * norm_center_dist)
            if score > best_score:
                best_score = score
                best_mask = m_u8

        return best_mask

    def run_rim_pipeline(self, img, target_tube, depth_prior_m=None, depth_prior_confidence=0.0):
        """按源码逻辑执行 Stage2: Crop -> Rim -> Mask -> Pose"""
        cx1, cy1, cx2, cy2 = get_centered_crop_coords(
            target_tube['bbox_xywh'],
            img.shape[1], img.shape[0],
            padding_ratio=cfg.CROP_PADDING_RATIO
        )
        crop_img = img[cy1:cy2, cx1:cx2]

        if crop_img.size == 0:
            return None, None, None, (cx1, cy1)

        results_rim = self.model_rim(crop_img, verbose=False, conf=0.4)

        best_rim_mask = None
        if results_rim and results_rim[0].masks is not None:
            masks_data = results_rim[0].masks.data.cpu().numpy()
            best_rim_mask = self._select_best_rim_mask(masks_data, crop_img.shape)

        if best_rim_mask is None:
            return None, None, None, (cx1, cy1)

        bbox_w = float(target_tube['bbox_xywh'][2])
        close_size = int(max(
            cfg.RIM_MASK_CLOSE_MIN,
            min(cfg.RIM_MASK_CLOSE_MAX, bbox_w * cfg.RIM_MASK_CLOSE_SCALE)
        ))
        if close_size % 2 == 0:
            close_size += 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_size, close_size))
        prepared_mask = cv2.morphologyEx(best_rim_mask, cv2.MORPH_CLOSE, kernel)

        global_rim_mask = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
        global_rim_mask[cy1:cy2, cx1:cx2] = prepared_mask
        global_body_mask = None
        if target_tube.get('raw_body_mask') is not None:
            resized_body_mask = cv2.resize(
                target_tube['raw_body_mask'],
                (img.shape[1], img.shape[0]),
                interpolation=cv2.INTER_LINEAR
            )
            global_body_mask = (resized_body_mask > 0.5).astype(np.uint8) * 255

        raw_pose, msg, _, roi_vis, roi_offset = self.pose_estimator.process_detection(
            img,
            global_rim_mask,
            (cx1, cy1),
            body_bbox=target_tube.get('bbox_xywh'),
            body_mask=global_body_mask,
            depth_prior_m=depth_prior_m,
            depth_prior_confidence=depth_prior_confidence
        )

        return raw_pose, global_rim_mask, roi_vis, roi_offset

    def project_point_3d_to_2d(self, point_3d: np.ndarray):
        """3D点投影到图像平面"""
        z = float(point_3d[2])
        if z <= 1e-6:
            return None

        u = int(self.intr.fx * point_3d[0] / z + self.intr.ppx)
        v = int(self.intr.fy * point_3d[1] / z + self.intr.ppy)
        return (u, v)

    def draw_pose_info(self, img, pose, is_target=False):
        """按源码风格绘制位姿信息"""
        if pose is None:
            return

        ellipse_color = (0, 255, 255) if is_target else (0, 0, 255)
        cv2.ellipse(img, pose['ellipse_2d'], ellipse_color, 2)

        c_3d = pose['center_3d']
        normal = pose['normal']
        p_center = self.project_point_3d_to_2d(c_3d)
        if p_center is None:
            return

        axis_len = 0.08
        p_z = self.project_point_3d_to_2d(c_3d + normal * axis_len)
        if p_z is not None:
            cv2.arrowedLine(img, p_center, p_z, (255, 0, 0), 3)

        angle_deg = pose['ellipse_2d'][2]
        ang_rad = math.radians(angle_deg)
        img_x_vec = np.array([math.cos(ang_rad), math.sin(ang_rad), 0.0])

        x_axis_3d = img_x_vec - np.dot(img_x_vec, normal) * normal
        if np.linalg.norm(x_axis_3d) > 1e-6:
            x_axis_3d /= np.linalg.norm(x_axis_3d)
        else:
            x_axis_3d = np.array([1.0, 0.0, 0.0])

        y_axis_3d = np.cross(normal, x_axis_3d)

        p_x = self.project_point_3d_to_2d(c_3d + x_axis_3d * axis_len)
        p_y = self.project_point_3d_to_2d(c_3d + y_axis_3d * axis_len)

        if p_x is not None:
            cv2.line(img, p_center, p_x, (0, 0, 255), 2)
        if p_y is not None:
            cv2.line(img, p_center, p_y, (0, 255, 0), 2)

        side_lines = pose.get('side_lines_2d', [])
        side_colors = [(255, 80, 220), (80, 255, 220)]
        for idx, line in enumerate(side_lines[:2]):
            p0 = tuple(np.round(line['p0_uv']).astype(int))
            p1 = tuple(np.round(line['p1_uv']).astype(int))
            color = side_colors[idx % len(side_colors)]
            cv2.line(img, p0, p1, color, 2)

        text = f"Z:{c_3d[2]:.3f}m"
        cv2.putText(img, text, (p_center[0] - 10, p_center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    def draw_target_pose_info(self, img, pose):
        """绘制计算得到的目标位姿坐标轴（仅可视化）"""
        if pose is None:
            return

        c_3d = pose['center_3d']
        normal = pose['normal'] / (np.linalg.norm(pose['normal']) + 1e-6)

        outward = normal
        inward = -outward
        camera_view_dir = inward if self.look_inward else outward

        # 目标相机位置：沿视线反方向退开 scan_distance
        p_cam_des = c_3d - camera_view_dir * self.scan_distance
        p_center = self.project_point_3d_to_2d(p_cam_des)
        if p_center is None:
            return

        cv2.circle(img, p_center, 6, (255, 0, 255), -1)
        cv2.putText(img, "target", (p_center[0] + 8, p_center[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        axis_len = 0.06
        z_axis = camera_view_dir
        x_axis = np.cross(np.array([0.0, 0.0, 1.0]), z_axis)
        if np.linalg.norm(x_axis) < 1e-6:
            x_axis = np.cross(np.array([0.0, 1.0, 0.0]), z_axis)
        x_axis = x_axis / (np.linalg.norm(x_axis) + 1e-6)
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / (np.linalg.norm(y_axis) + 1e-6)

        p_x = self.project_point_3d_to_2d(p_cam_des + x_axis * axis_len)
        p_y = self.project_point_3d_to_2d(p_cam_des + y_axis * axis_len)
        p_z = self.project_point_3d_to_2d(p_cam_des + z_axis * axis_len)

        if p_x is not None:
            cv2.line(img, p_center, p_x, (128, 0, 255), 2)
        if p_y is not None:
            cv2.line(img, p_center, p_y, (255, 0, 128), 2)
        if p_z is not None:
            cv2.arrowedLine(img, p_center, p_z, (255, 0, 255), 2)

    def draw_detection_box(self, img, tube_info, is_target=False, is_candidate=False):
        """按源码风格绘制检测框"""
        cx, cy, w, h = tube_info['bbox_xywh']
        x1, y1 = int(cx - w / 2), int(cy - h / 2)
        x2, y2 = int(cx + w / 2), int(cy + h / 2)

        if is_target:
            color = (0, 255, 0)
            thickness = 3
        elif is_candidate:
            color = (0, 200, 255)
            thickness = 2
        else:
            color = (50, 50, 50)
            thickness = 1

        cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
        label = f"ID:{tube_info['track_id']}"
        if is_target:
            label += " [SELECTED]"
        elif is_candidate:
            label += " [CANDIDATE]"

        cv2.putText(img, label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    def draw_segmentation_masks(self, img, body_mask=None, rim_mask=None, alpha=0.4):
        """按源码风格叠加管身/管口 mask"""
        if body_mask is None and rim_mask is None:
            return

        overlay = img.copy()
        if body_mask is not None:
            overlay[body_mask > 0] = (255, 150, 0)
        if rim_mask is not None:
            overlay[rim_mask > 0] = (0, 0, 255)

        cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)
    
    def on_target_selected(self, msg: Int32):
        """处理目标选择请求"""
        self.selected_target_id = msg.data
        self.ui_candidate_id = msg.data
        self.target_pose_sent = False
        self.get_logger().info(f"目标已更新: 圆柱 ID = {self.selected_target_id}")

    def handle_ui_key(self, key: int, detected_tubes):
        """处理可视化窗口键盘输入"""
        if key == 255:
            return

        if len(detected_tubes) == 0:
            if key in (ord('x'), ord('X')):
                self.selected_target_id = None
            return

        ids = [t['track_id'] for t in detected_tubes]
        if self.ui_candidate_id not in ids:
            self.ui_candidate_id = ids[0]

        idx = ids.index(self.ui_candidate_id)

        if key in (ord('a'), ord('A')):
            idx = (idx - 1) % len(ids)
            self.ui_candidate_id = ids[idx]
        elif key in (ord('d'), ord('D')):
            idx = (idx + 1) % len(ids)
            self.ui_candidate_id = ids[idx]
        elif key in (13, ord('c'), ord('C')):
            self.selected_target_id = self.ui_candidate_id
            self.target_pose_sent = False
            msg = Int32()
            msg.data = int(self.selected_target_id)
            self.target_id_pub.publish(msg)
            self.get_logger().info(f"✓ 已确认目标圆柱: ID={self.selected_target_id}")
        elif key in (ord('x'), ord('X')):
            self.selected_target_id = None
            self.target_pose_sent = False
            self.get_logger().info("已取消目标选择")
    
    def destroy_node(self):
        """清理资源"""
        try:
            self.pipeline.stop()
        except Exception:
            pass
        if self.enable_visualization:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CylinderDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
