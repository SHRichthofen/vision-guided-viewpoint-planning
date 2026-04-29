#!/usr/bin/env python3
"""
控制层节点 - 单次定位测试版

功能：
  1. 订阅目标 ID（/target_id）
  2. 订阅转换层发布的圆柱中心/轴向语义（/cylinder_semantics_base）
  3. 基于“中心 + 轴向”直接生成单个观察位姿
  4. 通过手眼标定转换为末端位姿并发布

说明：
  - 每次选中目标后，只接受第一帧 /cylinder_semantics_base
  - 只发布一次 /target_pose 与 /target_pose_stamped
  - 后续视觉更新一律忽略，直到下一次 /target_id 到来
  - 这个版本用于直接测试单次定位精度，不做二次修正
"""

import json
import os
from typing import Optional, Tuple

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformException, TransformListener


def _default_calib_file():
    try:
        pkg_share_dir = get_package_share_directory('hand_eye_calibration')
        return os.path.join(pkg_share_dir, 'calib.yaml')
    except Exception:
        return ''


class ArmPoseController(Node):
    def __init__(self):
        super().__init__('arm_pose_controller')

        self.get_logger().info('=' * 60)
        self.get_logger().info('控制层节点启动 (Arm Pose Controller - Single Shot Mode)')
        self.get_logger().info('=' * 60)

        self.declare_parameter('enable_debug', True)
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('calib_file', _default_calib_file())
        self.declare_parameter('desired_camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('calibration_camera_frame', 'camera_link')
        self.declare_parameter('use_tf_camera_frame_alignment', True)
        self.declare_parameter('roll_reference_axis', 'world_z')
        # 当无法从当前相机实际位置判断“朝管内”的方向时，使用这个符号作为后备方向。
        self.declare_parameter('axis_into_tube_sign', 1.0)
        # 观察相机相对管口中心沿 view_axis 反方向退开的距离。
        # 例：0.10 表示相机中心停在距管口中心 10 cm 的外侧。
        self.declare_parameter('camera_standoff_m', 0.10)

        self.debug = bool(self.get_parameter('enable_debug').value)
        self.target_frame = self.get_parameter('target_frame').value
        self.calib_file = self.get_parameter('calib_file').value
        self.desired_camera_frame = self.get_parameter('desired_camera_frame').value
        self.calibration_camera_frame = self.get_parameter('calibration_camera_frame').value
        self.use_tf_camera_frame_alignment = bool(self.get_parameter('use_tf_camera_frame_alignment').value)
        self.roll_reference_axis = self.get_parameter('roll_reference_axis').value
        self.axis_into_tube_sign = float(self.get_parameter('axis_into_tube_sign').value)
        self.camera_standoff_m = float(self.get_parameter('camera_standoff_m').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_frame_alignment_logged = False
        self.current_camera_pose_lookup_warned = False

        self.T_tool_to_camera = np.eye(4)
        self.load_calibration(self.calib_file)

        self.selected_target_id: Optional[int] = None
        self.latest_cylinder_measurement: Optional[dict] = None
        self.target_pose_sent = False

        self.cylinder_semantics_sub = self.create_subscription(
            String, '/cylinder_semantics_base', self.on_cylinder_semantics_callback, 10
        )
        self.target_id_sub = self.create_subscription(
            Int32, '/target_id', self.on_target_id_callback, 10
        )

        self.target_pose_pub = self.create_publisher(Pose, '/target_pose', 10)
        self.target_pose_stamped_pub = self.create_publisher(PoseStamped, '/target_pose_stamped', 10)
        self.debug_pub = self.create_publisher(String, '/target_pose_debug', 10)

        self.get_logger().info('✓ 控制层单次定位版初始化成功')
        self.get_logger().info('  mode=single_shot_first_pose_per_selection')
        self.get_logger().info(
            f'  camera_standoff_m={self.camera_standoff_m:.4f}, '
            f'axis_into_tube_sign(fallback)={self.axis_into_tube_sign:.2f}'
        )
        self.get_logger().info(
            f'  desired_camera_frame={self.desired_camera_frame}, '
            f'calibration_camera_frame={self.calibration_camera_frame}'
        )

    def load_calibration(self, calib_file: str):
        try:
            calib_data = None
            if os.path.exists(calib_file):
                with open(calib_file, 'r', encoding='utf-8') as f:
                    calib_data = yaml.safe_load(f)
            else:
                pkg_share_dir = get_package_share_directory('hand_eye_calibration')
                alt_path = os.path.join(pkg_share_dir, 'calib.yaml')
                if os.path.exists(alt_path):
                    with open(alt_path, 'r', encoding='utf-8') as f:
                        calib_data = yaml.safe_load(f)

            if calib_data is None:
                self.get_logger().warn(f'未找到标定文件，使用单位矩阵: {calib_file}')
                self.T_tool_to_camera = np.eye(4)
                return

            trans = calib_data['tool_to_camera']['translation']
            rot = calib_data['tool_to_camera']['rotation']
            t = np.array([trans['x'], trans['y'], trans['z']], dtype=float)
            q = np.array([rot['x'], rot['y'], rot['z'], rot['w']], dtype=float)
            q = q / max(np.linalg.norm(q), 1e-9)

            self.T_tool_to_camera = np.eye(4)
            self.T_tool_to_camera[:3, :3] = R.from_quat(q).as_matrix()
            self.T_tool_to_camera[:3, 3] = t
            self.get_logger().info('✓ 控制层加载标定成功（T_tool_to_camera）')
        except Exception as e:
            self.get_logger().warn(f'加载标定失败，使用单位矩阵: {e}')
            self.T_tool_to_camera = np.eye(4)

    def on_target_id_callback(self, msg: Int32):
        self.selected_target_id = int(msg.data)
        # 切换目标后重新等待“新一帧” base 系观测，避免把旧缓存误配到新 ID 上。
        self.latest_cylinder_measurement = None
        self.target_pose_sent = False
        self.get_logger().info(f'🎯 目标已切换: 圆柱 ID = {self.selected_target_id}')

    def on_cylinder_semantics_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'解析 /cylinder_semantics_base 失败: {e}')
            return

        center = payload.get('top_center_base')
        axis = payload.get('axis_base')
        if center is None or axis is None:
            self.get_logger().warn('转换层语义缺少 top_center_base 或 axis_base，跳过本帧')
            return

        self.latest_cylinder_measurement = payload
        if self.selected_target_id is None:
            return
        if self.target_pose_sent:
            return
        self.generate_and_publish_target(trigger_reason='first_pose_after_selection')

    def _get_reference_axis(self) -> np.ndarray:
        if self.roll_reference_axis == 'world_x':
            return np.array([1.0, 0.0, 0.0], dtype=float)
        if self.roll_reference_axis == 'world_y':
            return np.array([0.0, 1.0, 0.0], dtype=float)
        return np.array([0.0, 0.0, 1.0], dtype=float)

    def _look_at_quaternion(
        self,
        camera_position: np.ndarray,
        target_point: np.ndarray,
    ) -> Optional[np.ndarray]:
        z_axis = target_point - camera_position
        z_norm = np.linalg.norm(z_axis)
        if z_norm < 1e-8:
            return None
        z_axis = z_axis / z_norm

        ref = self._get_reference_axis()
        x_axis = ref - np.dot(ref, z_axis) * z_axis
        if np.linalg.norm(x_axis) < 1e-6:
            ref = np.array([0.0, 1.0, 0.0], dtype=float)
            x_axis = ref - np.dot(ref, z_axis) * z_axis
        x_axis = x_axis / max(np.linalg.norm(x_axis), 1e-9)
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / max(np.linalg.norm(y_axis), 1e-9)
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / max(np.linalg.norm(x_axis), 1e-9)

        q = R.from_matrix(np.column_stack([x_axis, y_axis, z_axis])).as_quat()
        return q / max(np.linalg.norm(q), 1e-9)

    def align_desired_camera_pose_to_calibration_frame(
        self,
        T_ref_cam_des: np.ndarray,
    ) -> Optional[np.ndarray]:
        if (not self.use_tf_camera_frame_alignment) or (
            self.desired_camera_frame == self.calibration_camera_frame
        ):
            return T_ref_cam_des
        try:
            tf_desired_from_calib = self.tf_buffer.lookup_transform(
                self.desired_camera_frame,
                self.calibration_camera_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
            q = np.array([
                tf_desired_from_calib.transform.rotation.x,
                tf_desired_from_calib.transform.rotation.y,
                tf_desired_from_calib.transform.rotation.z,
                tf_desired_from_calib.transform.rotation.w,
            ], dtype=float)
            t = np.array([
                tf_desired_from_calib.transform.translation.x,
                tf_desired_from_calib.transform.translation.y,
                tf_desired_from_calib.transform.translation.z,
            ], dtype=float)
            q = q / max(np.linalg.norm(q), 1e-9)
            T_desired_calib = np.eye(4)
            T_desired_calib[:3, :3] = R.from_quat(q).as_matrix()
            T_desired_calib[:3, 3] = t
            if not self.camera_frame_alignment_logged:
                self.get_logger().info(
                    f'✓ 相机帧对齐已启用: {self.desired_camera_frame} <- {self.calibration_camera_frame}'
                )
                self.camera_frame_alignment_logged = True
            return T_ref_cam_des @ T_desired_calib
        except TransformException as e:
            self.get_logger().warn(f'相机帧对齐 TF 查询失败: {e}')
            return None

    def _tool_pose_from_camera_pose(self, T_ref_cam: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        T_ref_tool = T_ref_cam @ np.linalg.inv(self.T_tool_to_camera)
        p = T_ref_tool[:3, 3]
        q = R.from_matrix(T_ref_tool[:3, :3]).as_quat()
        if not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)):
            return None
        q = q / max(np.linalg.norm(q), 1e-9)
        return p, q

    def _lookup_current_camera_position(self) -> Optional[np.ndarray]:
        try:
            tf_target_from_camera = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.desired_camera_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
            return np.array([
                tf_target_from_camera.transform.translation.x,
                tf_target_from_camera.transform.translation.y,
                tf_target_from_camera.transform.translation.z,
            ], dtype=float)
        except TransformException as e:
            if not self.current_camera_pose_lookup_warned:
                self.get_logger().warn(f'查询当前相机位姿失败，退回固定轴向符号: {e}')
                self.current_camera_pose_lookup_warned = True
            return None

    def _resolve_view_axis(self, center: np.ndarray, axis_raw: np.ndarray) -> np.ndarray:
        axis = axis_raw / max(np.linalg.norm(axis_raw), 1e-9)
        current_camera_position = self._lookup_current_camera_position()
        if current_camera_position is None:
            return axis * self.axis_into_tube_sign

        camera_side = current_camera_position - center
        camera_side_norm = np.linalg.norm(camera_side)
        if camera_side_norm < 1e-6:
            return axis * self.axis_into_tube_sign
        camera_side = camera_side / camera_side_norm

        same_score = float(np.dot(camera_side, -axis))
        flipped_score = float(np.dot(camera_side, axis))
        if flipped_score > same_score:
            axis = -axis
        return axis

    def _compute_target_camera_pose(
        self,
        center: np.ndarray,
        axis_raw: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        view_axis = self._resolve_view_axis(center, axis_raw)
        # 单次定位测试模式只保留一个最直接的几何策略：
        # 相机从管口中心沿朝外方向退开固定距离，再直接看向管口中心本身。
        camera_position = center - view_axis * self.camera_standoff_m
        look_at_point = center
        return camera_position, look_at_point, view_axis

    def _publish_debug_report(self, report: dict):
        msg = String()
        msg.data = json.dumps(report, ensure_ascii=False)
        self.debug_pub.publish(msg)

    def generate_and_publish_target(self, trigger_reason: str = 'unknown'):
        if self.target_pose_sent:
            return
        measurement = self.latest_cylinder_measurement
        if measurement is None:
            return

        center = np.asarray(measurement.get('top_center_base'), dtype=float)
        axis_raw = np.asarray(measurement.get('axis_base'), dtype=float)
        if center.shape != (3,) or axis_raw.shape != (3,):
            self.get_logger().warn('转换层语义中的中心或轴向维度非法，跳过本次发布')
            return
        axis_raw = axis_raw / max(np.linalg.norm(axis_raw), 1e-9)

        camera_position, look_at_point, view_axis = self._compute_target_camera_pose(center, axis_raw)
        q_cam = self._look_at_quaternion(camera_position, look_at_point)
        if q_cam is None:
            self.get_logger().warn('目标相机姿态求解失败，跳过本次发布')
            return

        T_ref_cam = np.eye(4)
        T_ref_cam[:3, :3] = R.from_quat(q_cam).as_matrix()
        T_ref_cam[:3, 3] = camera_position
        T_ref_cam = self.align_desired_camera_pose_to_calibration_frame(T_ref_cam)
        if T_ref_cam is None:
            return

        tool_pose = self._tool_pose_from_camera_pose(T_ref_cam)
        if tool_pose is None:
            self.get_logger().warn('末端位姿求解失败，跳过本次发布')
            return

        tool_position, tool_quaternion = tool_pose

        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        target_pose_stamped.header.frame_id = self.target_frame
        target_pose_stamped.pose.position.x = float(tool_position[0])
        target_pose_stamped.pose.position.y = float(tool_position[1])
        target_pose_stamped.pose.position.z = float(tool_position[2])
        target_pose_stamped.pose.orientation.x = float(tool_quaternion[0])
        target_pose_stamped.pose.orientation.y = float(tool_quaternion[1])
        target_pose_stamped.pose.orientation.z = float(tool_quaternion[2])
        target_pose_stamped.pose.orientation.w = float(tool_quaternion[3])

        self.target_pose_pub.publish(target_pose_stamped.pose)
        self.target_pose_stamped_pub.publish(target_pose_stamped)
        self.target_pose_sent = True

        report = {
            'selected_target_id': self.selected_target_id,
            'mode': 'single_shot_first_pose_per_selection',
            'trigger_reason': trigger_reason,
            'measurement_kind': measurement.get('measurement_kind', 'center_axis'),
            'center_base': [float(v) for v in center],
            'axis_raw_base': [float(v) for v in axis_raw],
            'view_axis_base': [float(v) for v in view_axis],
            'camera_position_base': [float(v) for v in camera_position],
            'look_at_point_base': [float(v) for v in look_at_point],
            'tool_position_base': [float(v) for v in tool_position],
            'tool_quaternion_base': [float(v) for v in tool_quaternion],
        }
        self._publish_debug_report(report)

        if self.debug:
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'📍 发布单次目标位姿 (ID={self.selected_target_id})')
            self.get_logger().info(
                f'  center(base) = [{center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f}]'
            )
            self.get_logger().info(
                f'  view_axis(base) = [{view_axis[0]:.4f}, {view_axis[1]:.4f}, {view_axis[2]:.4f}]'
            )
            self.get_logger().info(
                f'  camera(base) = [{camera_position[0]:.4f}, {camera_position[1]:.4f}, {camera_position[2]:.4f}]'
            )
            self.get_logger().info(
                f'  tool(base) = [{tool_position[0]:.4f}, {tool_position[1]:.4f}, {tool_position[2]:.4f}]'
            )
            self.get_logger().info('✓ 已发布一次目标位姿；本轮后续视觉更新将被忽略')
            self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = ArmPoseController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
