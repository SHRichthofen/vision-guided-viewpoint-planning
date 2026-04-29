#!/usr/bin/env python3
"""
转换层节点 - 坐标系转换
功能：
  1. 接收视觉系统检测的圆柱位姿（相机坐标系）
    2. 使用TF树进行坐标转换（camera → base_link）
  3. 发布转换后的圆柱位姿（base_link坐标系）

发布话题：
  /cylinder_pose_base (geometry_msgs/PoseStamped): 转换后的圆柱口中心（orientation 为占位值）
  /cylinder_semantics_base (std_msgs/String): 转换后的圆柱口中心与轴向语义
  
订阅话题：
  /vision/selected_cylinder_semantics (std_msgs/String): 相机坐标系中的检测结果
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32, String
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import json
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformException


def _default_calib_file():
    try:
        pkg_share_dir = get_package_share_directory('hand_eye_calibration')
        return os.path.join(pkg_share_dir, 'calib.yaml')
    except Exception:
        return ''


class VisionToArmTransform(Node):
    def __init__(self):
        super().__init__('vision_to_arm_transform')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("转换层节点启动 (Vision-to-Arm Transform)")
        self.get_logger().info("=" * 60)

        # ==================== 参数声明 ====================
        self.declare_parameter('calib_file', _default_calib_file())
        self.declare_parameter('enable_debug', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_link')
        # 末端参考帧需与URDF link一致
        self.declare_parameter('effector_frame', 'tcp_link')
        self.declare_parameter('publish_camera_link_to_optical_tf', True)
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('prefer_runtime_camera_optical_tf', True)
        self.declare_parameter('runtime_camera_optical_tf_wait_sec', 2.5)
        # 以下参数来自当前运行中的 RealSense TF 读取结果（camera_link -> camera_color_optical_frame）
        self.declare_parameter('camera_link_to_optical_tx', -0.00019149143190588802)
        self.declare_parameter('camera_link_to_optical_ty', 0.014909240417182446)
        self.declare_parameter('camera_link_to_optical_tz', 0.00011411493323976174)
        self.declare_parameter('camera_link_to_optical_qx', -0.50204567)
        self.declare_parameter('camera_link_to_optical_qy', 0.50124772)
        self.declare_parameter('camera_link_to_optical_qz', -0.49795181)
        self.declare_parameter('camera_link_to_optical_qw', 0.49874328)
        
        # ==================== 获取参数 ====================
        calib_file = self.get_parameter('calib_file').value
        self.debug = self.get_parameter('enable_debug').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.effector_frame = self.get_parameter('effector_frame').value
        self.publish_camera_link_to_optical_tf = self.get_parameter('publish_camera_link_to_optical_tf').value
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        self.prefer_runtime_camera_optical_tf = bool(self.get_parameter('prefer_runtime_camera_optical_tf').value)
        self.runtime_camera_optical_tf_wait_sec = float(self.get_parameter('runtime_camera_optical_tf_wait_sec').value)
        self.camera_link_to_optical_tx = float(self.get_parameter('camera_link_to_optical_tx').value)
        self.camera_link_to_optical_ty = float(self.get_parameter('camera_link_to_optical_ty').value)
        self.camera_link_to_optical_tz = float(self.get_parameter('camera_link_to_optical_tz').value)
        self.camera_link_to_optical_qx = float(self.get_parameter('camera_link_to_optical_qx').value)
        self.camera_link_to_optical_qy = float(self.get_parameter('camera_link_to_optical_qy').value)
        self.camera_link_to_optical_qz = float(self.get_parameter('camera_link_to_optical_qz').value)
        self.camera_link_to_optical_qw = float(self.get_parameter('camera_link_to_optical_qw').value)

        # 目标选择状态（用于“仅在选定时输出一次”）
        self.selected_target_id = None
        self.last_logged_target_id = None
        
        # ==================== 加载手眼标定结果 ====================
        self.T_tool_to_camera = None
        self.load_calibration(calib_file)
        
        if self.T_tool_to_camera is None:
            self.get_logger().error("✗ 标定矩阵加载失败! 使用单位矩阵")
            self.T_tool_to_camera = np.eye(4)

        # ==================== TF 初始化 ====================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.runtime_camera_optical_tf_loaded = False
        if self.prefer_runtime_camera_optical_tf:
            self.try_load_runtime_camera_optical_tf(self.runtime_camera_optical_tf_wait_sec)
        # 旧逻辑（保留注释）：分两次发送静态TF，晚加入订阅者可能只收到最后一次
        # self.publish_camera_static_tf()
        # self.publish_camera_link_to_optical_static_tf()
        # 新逻辑：一次性成组发送所有静态TF，确保链路完整可重放
        self.publish_static_tfs_bundle()
        
        # ==================== 订阅/发布 ====================
        # 订阅视觉系统的圆柱语义（相机坐标系）
        self.cylinder_semantics_sub = self.create_subscription(
            String,
            '/vision/selected_cylinder_semantics',
            self.on_cylinder_semantics_callback,
            10
        )

        # 订阅当前选中目标ID（仅用于日志节流）
        self.target_id_sub = self.create_subscription(
            Int32,
            '/target_id',
            self.on_target_id_callback,
            10
        )

        # 旧逻辑（保留注释）：订阅 /tool_pose 手工计算 base<-tool
        # self.tool_pose_sub = self.create_subscription(
        #     PoseStamped,
        #     '/tool_pose',
        #     self.on_tool_pose_callback,
        #     10
        # )
        
        # 发布转换后的圆柱中心（兼容旧接口）
        self.cylinder_pose_base_pub = self.create_publisher(
            PoseStamped,
            '/cylinder_pose_base',
            10
        )
        self.cylinder_semantics_base_pub = self.create_publisher(
            String,
            '/cylinder_semantics_base',
            10
        )

        # 旧逻辑（保留注释）：动态位姿 T_base_to_tool（来自 /tool_pose）
        # self.T_base_to_tool = None
        
        self.get_logger().info("✓ 转换层节点初始化成功")
        self.get_logger().info(f"✓ 手眼标定矩阵已加载:")
        self.get_logger().info(f"  T_tool_to_camera =\n{self.T_tool_to_camera}\n")
        self.get_logger().info(f"  base_frame={self.base_frame}, effector_frame={self.effector_frame}, camera_frame={self.camera_frame}")

    def publish_camera_static_tf(self):
        """
        发布静态手眼TF：parent=effector_frame, child=camera_frame
        变换内容使用 T_tool_to_camera，与 easy_handeye2 发布语义保持一致
        """
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.effector_frame
        tf_msg.child_frame_id = self.camera_frame

        t = self.T_tool_to_camera[:3, 3]
        q = R.from_matrix(self.T_tool_to_camera[:3, :3]).as_quat()

        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])
        tf_msg.transform.rotation.x = float(q[0])
        tf_msg.transform.rotation.y = float(q[1])
        tf_msg.transform.rotation.z = float(q[2])
        tf_msg.transform.rotation.w = float(q[3])

        self.static_tf_broadcaster.sendTransform(tf_msg)
        self.get_logger().info(
            f"✓ 已发布静态TF: {self.effector_frame} -> {self.camera_frame}"
        )

    def build_camera_static_tf(self) -> TransformStamped:
        """构建静态手眼TF：parent=effector_frame, child=camera_frame（使用 T_tool_to_camera）"""
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.effector_frame
        tf_msg.child_frame_id = self.camera_frame

        t = self.T_tool_to_camera[:3, 3]
        q = R.from_matrix(self.T_tool_to_camera[:3, :3]).as_quat()

        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])
        tf_msg.transform.rotation.x = float(q[0])
        tf_msg.transform.rotation.y = float(q[1])
        tf_msg.transform.rotation.z = float(q[2])
        tf_msg.transform.rotation.w = float(q[3])
        return tf_msg

    def build_camera_link_to_optical_static_tf(self) -> TransformStamped:
        """构建 camera_link -> camera_color_optical_frame 静态TF"""
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.camera_frame
        tf_msg.child_frame_id = self.camera_optical_frame
        tf_msg.transform.translation.x = self.camera_link_to_optical_tx
        tf_msg.transform.translation.y = self.camera_link_to_optical_ty
        tf_msg.transform.translation.z = self.camera_link_to_optical_tz
        tf_msg.transform.rotation.x = self.camera_link_to_optical_qx
        tf_msg.transform.rotation.y = self.camera_link_to_optical_qy
        tf_msg.transform.rotation.z = self.camera_link_to_optical_qz
        tf_msg.transform.rotation.w = self.camera_link_to_optical_qw
        return tf_msg

    def try_load_runtime_camera_optical_tf(self, wait_sec: float):
        """尝试从运行中的D435i TF读取 camera_link -> camera_optical 变换。"""
        if wait_sec <= 0.0:
            return

        end_time = self.get_clock().now() + Duration(seconds=float(wait_sec))
        while self.get_clock().now() < end_time:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    self.camera_optical_frame,
                    Time(),
                    timeout=Duration(seconds=0.15)
                )
                self.camera_link_to_optical_tx = float(tf_msg.transform.translation.x)
                self.camera_link_to_optical_ty = float(tf_msg.transform.translation.y)
                self.camera_link_to_optical_tz = float(tf_msg.transform.translation.z)
                self.camera_link_to_optical_qx = float(tf_msg.transform.rotation.x)
                self.camera_link_to_optical_qy = float(tf_msg.transform.rotation.y)
                self.camera_link_to_optical_qz = float(tf_msg.transform.rotation.z)
                self.camera_link_to_optical_qw = float(tf_msg.transform.rotation.w)
                self.runtime_camera_optical_tf_loaded = True
                self.get_logger().info(
                    f"✓ 已读取运行时TF: {self.camera_frame} -> {self.camera_optical_frame}"
                )
                return
            except TransformException:
                continue

        self.get_logger().warn(
            f"未在 {wait_sec:.1f}s 内读取到运行时TF: {self.camera_frame} -> {self.camera_optical_frame}，回退到参数中的固定值。"
        )

    def publish_static_tfs_bundle(self):
        """
        一次性发布静态TF集合，避免分多次发送导致静态历史不完整。
        """
        tf_list = [self.build_camera_static_tf()]

        if self.publish_camera_link_to_optical_tf:
            if self.runtime_camera_optical_tf_loaded:
                self.get_logger().info(
                    f"✓ 使用运行时TF: {self.camera_frame} -> {self.camera_optical_frame}，跳过静态发布"
                )
                self.static_tf_broadcaster.sendTransform(tf_list)
                self.get_logger().info(
                    f"✓ 已批量发布静态TF数量: {len(tf_list)}"
                )
                return

            try:
                already_exists = self.tf_buffer.can_transform(
                    self.camera_frame,
                    self.camera_optical_frame,
                    Time(),
                    timeout=Duration(seconds=0.2)
                )
            except Exception:
                already_exists = False

            if already_exists:
                self.get_logger().info(
                    f"✓ 检测到已有TF: {self.camera_frame} -> {self.camera_optical_frame}，跳过重复发布"
                )
            else:
                tf_list.append(self.build_camera_link_to_optical_static_tf())

        self.static_tf_broadcaster.sendTransform(tf_list)
        self.get_logger().info(
            f"✓ 已批量发布静态TF数量: {len(tf_list)}"
        )

    def publish_camera_link_to_optical_static_tf(self):
        """
        发布（或检测后跳过）camera_link -> camera_color_optical_frame 静态TF。
        当 RealSense 节点已发布同名TF时，避免重复发布。
        """
        if not self.publish_camera_link_to_optical_tf:
            return

        if self.runtime_camera_optical_tf_loaded:
            self.get_logger().info(
                f"✓ 使用运行时TF: {self.camera_frame} -> {self.camera_optical_frame}，跳过静态发布"
            )
            return

        try:
            already_exists = self.tf_buffer.can_transform(
                self.camera_frame,
                self.camera_optical_frame,
                Time(),
                timeout=Duration(seconds=0.2)
            )
        except Exception:
            already_exists = False

        if already_exists:
            self.get_logger().info(
                f"✓ 检测到已有TF: {self.camera_frame} -> {self.camera_optical_frame}，跳过重复发布"
            )
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.camera_frame
        tf_msg.child_frame_id = self.camera_optical_frame
        tf_msg.transform.translation.x = self.camera_link_to_optical_tx
        tf_msg.transform.translation.y = self.camera_link_to_optical_ty
        tf_msg.transform.translation.z = self.camera_link_to_optical_tz
        tf_msg.transform.rotation.x = self.camera_link_to_optical_qx
        tf_msg.transform.rotation.y = self.camera_link_to_optical_qy
        tf_msg.transform.rotation.z = self.camera_link_to_optical_qz
        tf_msg.transform.rotation.w = self.camera_link_to_optical_qw
        self.static_tf_broadcaster.sendTransform(tf_msg)

        self.get_logger().info(
            f"✓ 已发布静态TF: {self.camera_frame} -> {self.camera_optical_frame}"
        )

    def on_target_id_callback(self, msg: Int32):
        """记录当前选中的目标ID，用于单次日志输出"""
        self.selected_target_id = msg.data
        self.last_logged_target_id = None

    def vector_axis_angles_deg(self, direction: np.ndarray):
        """
        将单位方向向量转换为它与 base 系正 X/Y/Z 轴的夹角（单位：度）。
        例如：
          angle_x = arccos(nx)
          angle_y = arccos(ny)
          angle_z = arccos(nz)
        """
        direction = np.asarray(direction, dtype=float)
        norm = np.linalg.norm(direction)
        if norm < 1e-9:
            return None
        direction = direction / norm
        return np.degrees(np.arccos(np.clip(direction, -1.0, 1.0)))

    def _set_placeholder_orientation(self, pose):
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
    
    def load_calibration(self, calib_file):
        """加载手眼标定结果 (YAML格式)"""
        try:
            # 首先尝试从指定路径加载
            if os.path.exists(calib_file):
                self.get_logger().info(f"从 {calib_file} 加载标定结果...")
                with open(calib_file, 'r') as f:
                    calib_data = yaml.safe_load(f)
            else:
                # 尝试从包目录加载
                self.get_logger().warn(f"路径 {calib_file} 不存在，尝试包目录...")
                pkg_share_dir = get_package_share_directory('hand_eye_calibration')
                alt_path = os.path.join(pkg_share_dir, 'config', 'calib.yaml')
                if os.path.exists(alt_path):
                    self.get_logger().info(f"从 {alt_path} 加载标定结果...")
                    with open(alt_path, 'r') as f:
                        calib_data = yaml.safe_load(f)
                else:
                    self.get_logger().error(f"✗ 标定文件未找到: {calib_file}")
                    return
            
            # 提取平移和旋转
            trans_dict = calib_data['tool_to_camera']['translation']
            rot_dict = calib_data['tool_to_camera']['rotation']
            
            t = np.array([trans_dict['x'], trans_dict['y'], trans_dict['z']], dtype=float)
            q = np.array([rot_dict['x'], rot_dict['y'], rot_dict['z'], rot_dict['w']], dtype=float)

            q_norm = np.linalg.norm(q)
            if q_norm < 1e-9:
                self.get_logger().error("✗ 标定四元数范数为0，使用单位旋转")
                q = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
            else:
                if abs(q_norm - 1.0) > 1e-3:
                    self.get_logger().warn(
                        f"⚠ 标定四元数未归一化 (norm={q_norm:.6f})，将自动归一化"
                    )
                q = q / q_norm
            
            # 构建齐次变换矩阵
            rotation_matrix = R.from_quat(q).as_matrix()
            self.T_tool_to_camera = np.eye(4)
            self.T_tool_to_camera[:3, :3] = rotation_matrix
            self.T_tool_to_camera[:3, 3] = t

            reproj_err = calib_data.get('metadata', {}).get('reprojection_error', None)
            if reproj_err is not None:
                self.get_logger().info(f"  重投影误差: {float(reproj_err):.6f} m")
                if float(reproj_err) > 0.05:
                    self.get_logger().warn(
                        "⚠ 重投影误差较大，坐标转换可能明显偏差。建议重新进行手眼标定。"
                    )
            
            self.get_logger().info(f"✓ 标定结果加载成功")
            self.get_logger().info(f"  平移 (mm): {t * 1000}")
            self.get_logger().info(f"  旋转四元数: {q}")
            
        except Exception as e:
            self.get_logger().error(f"✗ 加载标定文件出错: {e}")
    
    def on_cylinder_semantics_callback(self, msg: String):
        """
        接收相机坐标系中的圆柱中心/轴向语义，使用TF转换为base坐标系
        """
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"解析 /vision/selected_cylinder_semantics 失败: {e}")
            return

        center_cam = payload.get('top_center_cam')
        axis_cam = payload.get('axis_cam')
        if center_cam is None or axis_cam is None:
            self.get_logger().warn("视觉语义缺少 top_center_cam 或 axis_cam，跳过本帧")
            return

        try:
            P_cylinder_camera = np.asarray(center_cam, dtype=float)
            direction_camera = np.asarray(axis_cam, dtype=float)
        except Exception:
            self.get_logger().warn("视觉语义中的中心或轴向格式非法，跳过本帧")
            return

        if P_cylinder_camera.shape != (3,) or direction_camera.shape != (3,):
            self.get_logger().warn("视觉语义中的中心或轴向维度非法，跳过本帧")
            return

        direction_norm = np.linalg.norm(direction_camera)
        if direction_norm < 1e-9:
            self.get_logger().warn("视觉语义中的轴向范数过小，跳过本帧")
            return
        direction_camera = direction_camera / direction_norm

        source_frame = payload.get('frame_id') or self.camera_optical_frame or self.camera_frame
        used_latest_tf = False
        stamp_sec = payload.get('stamp_sec', None)
        query_time = Time()
        if stamp_sec is not None:
            try:
                query_time = Time(nanoseconds=int(float(stamp_sec) * 1e9))
            except Exception:
                query_time = Time()

        try:
            tf_base_from_cam = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                query_time,
                timeout=Duration(seconds=0.1)
            )
        except TransformException as e:
            err = str(e)
            # 时间戳略超前时，回退到“最新可用TF”避免 future extrapolation
            if 'extrapolation into the future' in err.lower():
                try:
                    tf_base_from_cam = self.tf_buffer.lookup_transform(
                        self.base_frame,
                        source_frame,
                        Time(),
                        timeout=Duration(seconds=0.1)
                    )
                    used_latest_tf = True
                except TransformException as e_latest:
                    self.get_logger().warn(
                        f"TF查询失败(回退最新TF也失败): {self.base_frame} <- {source_frame}, 错误: {str(e_latest)}"
                    )
                    return
            else:
                self.get_logger().warn(
                    f"TF查询失败: {self.base_frame} <- {source_frame}, 错误: {err}"
                )
                return
        
        # ==================== 2. 使用TF转换到base坐标系 ====================
        R_base_from_cam = R.from_quat([
            tf_base_from_cam.transform.rotation.x,
            tf_base_from_cam.transform.rotation.y,
            tf_base_from_cam.transform.rotation.z,
            tf_base_from_cam.transform.rotation.w,
        ]).as_matrix()
        t_base_from_cam = np.array([
            tf_base_from_cam.transform.translation.x,
            tf_base_from_cam.transform.translation.y,
            tf_base_from_cam.transform.translation.z,
        ], dtype=float)

        P_cylinder_base = R_base_from_cam @ P_cylinder_camera + t_base_from_cam
        N_cylinder_base = R_base_from_cam @ direction_camera
        N_cylinder_base = N_cylinder_base / (np.linalg.norm(N_cylinder_base) + 1e-6)
        
        # ==================== 3. 发布中心点兼容接口 ====================
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.pose.position.x = float(P_cylinder_base[0])
        pose_stamped.pose.position.y = float(P_cylinder_base[1])
        pose_stamped.pose.position.z = float(P_cylinder_base[2])
        self._set_placeholder_orientation(pose_stamped.pose)
        self.cylinder_pose_base_pub.publish(pose_stamped)

        out_payload = dict(payload)
        out_payload['frame_id'] = self.base_frame
        out_payload['source_frame_id'] = source_frame
        out_payload['top_center_base'] = [float(v) for v in P_cylinder_base]
        out_payload['axis_base'] = [float(v) for v in N_cylinder_base]
        out_payload['transform_stamp_sec'] = float(self.get_clock().now().nanoseconds) * 1e-9
        semantics_msg = String()
        semantics_msg.data = json.dumps(out_payload, ensure_ascii=False)
        self.cylinder_semantics_base_pub.publish(semantics_msg)


        # 仅在目标“选定后”输出一次
        if self.debug and self.selected_target_id is not None and self.last_logged_target_id != self.selected_target_id:
            axis_angles_deg = self.vector_axis_angles_deg(N_cylinder_base)
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"✓ 已选定目标: ID={self.selected_target_id}")
            self.get_logger().info(f"  源坐标系: {source_frame} -> 目标坐标系: {self.base_frame}")
            if used_latest_tf:
                self.get_logger().info("  TF时间策略: 使用最新可用TF（已回退，避免future extrapolation）")
            self.get_logger().info(f"  圆柱中心(base): [{P_cylinder_base[0]:.4f}, {P_cylinder_base[1]:.4f}, {P_cylinder_base[2]:.4f}]")
            self.get_logger().info(f"  法向量(base):   [{N_cylinder_base[0]:.4f}, {N_cylinder_base[1]:.4f}, {N_cylinder_base[2]:.4f}]")
            if axis_angles_deg is not None:
                self.get_logger().info(
                    f"  与XYZ轴夹角(度): [X={axis_angles_deg[0]:.2f}, "
                    f"Y={axis_angles_deg[1]:.2f}, Z={axis_angles_deg[2]:.2f}]"
                )
            self.get_logger().info("=" * 60 + "\n")
            self.last_logged_target_id = self.selected_target_id

    def on_tool_pose_callback(self, msg: PoseStamped):
        """接收 /tool_pose，更新 T_base_to_tool """
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
        q = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ], dtype=float)

        T = np.eye(4)
        T[:3, :3] = R.from_quat(q).as_matrix()
        T[:3, 3] = p
        self.T_base_to_tool = T
    
def main(args=None):
    rclpy.init(args=args)
    node = VisionToArmTransform()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
