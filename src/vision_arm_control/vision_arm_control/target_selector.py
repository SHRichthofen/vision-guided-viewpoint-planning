#!/usr/bin/env python3
"""
目标选择器节点 - UI + 队列管理 + 状态监听
功能：
  1. 订阅所有检测到的圆柱（/vision/cylinders）
  2. 订阅相机RGB图像（/camera/color/image_raw）
  3. 监听机械臂运动状态（/joint_states）
  4. 在实时相机画面中显示所有检测到的圆柱和位姿
  5. 提供键盘交互选择目标
  6. 管理执行队列
  7. 发布选中目标ID（/target_id）

发布话题：
  /target_id (std_msgs/Int32): 当前选中的目标圆柱ID

订阅话题：
  /vision/cylinders (geometry_msgs/PoseArray): 所有检测到的圆柱
  /camera/color/image_raw (sensor_msgs/Image): 相机RGB图像
  /joint_states (sensor_msgs/JointState): 机械臂关节状态
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import numpy as np
import cv2
import threading
import time
import math


class TargetSelector(Node):
    def __init__(self):
        super().__init__('target_selector')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("目标选择器启动 (Target Selector)")
        self.get_logger().info("=" * 60)
        
        # ==================== 参数声明 ====================
        self.declare_parameter('enable_ui', True)
        self.declare_parameter('ui_scale', 1.0)
        self.declare_parameter('auto_mode', False)  # 自动队列模式
        self.declare_parameter('arm_reach_threshold', 0.01)  # 10mm - 判定到位阈值
        self.declare_parameter('arm_timeout_seconds', 10.0)  # 超时保护
        
        # ==================== 获取参数 ====================
        self.enable_ui = self.get_parameter('enable_ui').value
        self.ui_scale = self.get_parameter('ui_scale').value
        self.auto_mode = self.get_parameter('auto_mode').value
        self.arm_reach_threshold = self.get_parameter('arm_reach_threshold').value
        self.arm_timeout = self.get_parameter('arm_timeout_seconds').value
        
        # ==================== 状态 ====================
        self.detected_cylinders = []  # list of {id, pose}
        self.selected_cylinder_idx = 0  # UI中选中的圆柱索引
        self.execution_queue = []  # 执行队列
        self.is_arm_moving = False
        self.last_move_start_time = None
        self.last_published_id = None
        
        # ==================== 相机参数 ====================
        self.current_frame = None  # 当前相机图像
        self.bridge = CvBridge()
        # 默认相机内参 (RealSense D435i)
        self.fx = 906.948
        self.fy = 905.906
        self.cx = 640.0
        self.cy = 360.0
        
        # ==================== 订阅/发布 ====================
        # 订阅所有检测到的圆柱
        self.cylinders_sub = self.create_subscription(
            PoseArray,
            '/vision/cylinders',
            self.on_cylinders_callback,
            10
        )
        
        # 订阅相机RGB图像
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.on_image_callback,
            10
        )
        
        # 订阅机械臂关节状态（用于监听运动状态）
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.on_joint_states_callback,
            10
        )
        
        # 发布选中的目标ID
        self.target_id_pub = self.create_publisher(
            Int32,
            '/target_id',
            10
        )
        
        # ==================== UI线程 ====================
        if self.enable_ui:
            self.ui_thread = threading.Thread(target=self.ui_loop, daemon=True)
            self.ui_thread.start()
        
        self.get_logger().info("✓ 目标选择器初始化成功")
        self.get_logger().info(f"  UI启用: {self.enable_ui}")
        self.get_logger().info(f"  自动模式: {self.auto_mode}")
        self.get_logger().info(f"  到位阈值: {self.arm_reach_threshold*1000:.1f}mm\n")
    
    def on_cylinders_callback(self, msg: PoseArray):
        """处理检测到的圆柱数组"""
        self.detected_cylinders = []
        for idx, pose in enumerate(msg.poses):
            self.detected_cylinders.append({
                'id': idx,
                'pose': pose,
                'timestamp': time.time()
            })
        
        # 索引越界保护
        if self.selected_cylinder_idx >= len(self.detected_cylinders):
            self.selected_cylinder_idx = 0
        
        # 自动模式下，初始化队列
        if self.auto_mode and len(self.execution_queue) == 0 and len(self.detected_cylinders) > 0:
            self.execution_queue = [cyl['id'] for cyl in self.detected_cylinders]
            self.get_logger().info(f"📋 队列已初始化: {self.execution_queue}")
        
        # 发布当前选中的目标ID
        self.publish_selected_target()
    
    def on_image_callback(self, msg: Image):
        """处理相机图像"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"图像转换失败: {e}")
    
    def on_joint_states_callback(self, msg: JointState):
        """监听机械臂关节状态，判断是否在运动"""
        # 简单启发式：如果速度不全为0，则认为在运动
        if msg.velocity:
            velocity_norm = np.sqrt(sum(v**2 for v in msg.velocity))
            self.is_arm_moving = velocity_norm > 0.01
            
            if self.is_arm_moving and self.last_move_start_time is None:
                self.last_move_start_time = time.time()
    
    def publish_selected_target(self):
        """发布选中的目标ID"""
        if len(self.detected_cylinders) == 0:
            return
        
        selected_id = self.detected_cylinders[self.selected_cylinder_idx]['id']
        
        if selected_id != self.last_published_id:
            msg = Int32()
            msg.data = selected_id
            self.target_id_pub.publish(msg)
            self.last_published_id = selected_id
    
    def project_point_to_image(self, point_3d):
        """将3D点投影到图像平面"""
        z = point_3d.z
        if z <= 0:
            return None
        
        x_img = int(self.fx * point_3d.x / z + self.cx)
        y_img = int(self.fy * point_3d.y / z + self.cy)
        
        return (x_img, y_img)
    
    def draw_cylinder_pose(self, frame, pose, cylinder_id, is_selected=False):
        """在图像上绘制圆柱和位姿"""
        # 投影圆柱中心到图像
        center_img = self.project_point_to_image(pose.position)
        if center_img is None:
            return
        
        # 中心点颜色：选中为绿色，未选中为蓝色
        color = (0, 255, 0) if is_selected else (255, 128, 0)
        thickness = 3 if is_selected else 2
        
        # 绘制中心点
        cv2.circle(frame, center_img, 10, color, thickness)
        
        # 绘制ID标签
        label = f"ID:{cylinder_id}" + (" [SEL]" if is_selected else "")
        cv2.putText(frame, label, (center_img[0] - 30, center_img[1] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # === 绘制3D坐标轴 ===
        if is_selected:
            axis_len = 0.1  # 10cm轴长
            
            # Z轴（法线方向）- 蓝色
            # 简单方式：沿着Z方向延伸
            from geometry_msgs.msg import Point
            z_point = Point()
            z_point.x = pose.position.x
            z_point.y = pose.position.y
            z_point.z = pose.position.z + axis_len
            z_img = self.project_point_to_image(z_point)
            
            if z_img:
                cv2.arrowedLine(frame, center_img, z_img, (255, 0, 0), 2, tipLength=0.2)
            
            # 距离信息
            distance_text = f"Z:{pose.position.z:.3f}m"
            cv2.putText(frame, distance_text, (center_img[0] + 15, center_img[1] + 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            # 位置信息
            pos_text = f"Pos:({pose.position.x:.3f},{pose.position.y:.3f})"
            cv2.putText(frame, pos_text, (center_img[0] + 15, center_img[1] + 45),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    
    def ui_loop(self):
        """实时相机画面UI线程"""
        window_name = "Target Selector - 圆柱位姿显示"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        while rclpy.ok():
            try:
                # 使用相机图像或创建黑色背景
                if self.current_frame is not None:
                    display_frame = self.current_frame.copy()
                else:
                    display_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                
                # ==================== 绘制所有检测到的圆柱 ====================
                for i, cyl in enumerate(self.detected_cylinders):
                    is_selected = (i == self.selected_cylinder_idx)
                    self.draw_cylinder_pose(display_frame, cyl['pose'], cyl['id'], is_selected)
                
                # ==================== 绘制信息面板 ====================
                # 左上角：标题和检测信息
                cv2.putText(display_frame, "Target Selector - Camera View", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(display_frame, f"Detected: {len(self.detected_cylinders)}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 255), 1)
                
                # 底部：控制提示
                control_y = display_frame.shape[0] - 80
                cv2.putText(display_frame, "Controls:", (10, control_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
                cv2.putText(display_frame, "W/S or UP/DOWN - Select | ENTER - Confirm", 
                           (10, control_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                cv2.putText(display_frame, "A - Auto Mode | Q - Quit", 
                           (10, control_y + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                
                # 右上角：当前选中信息
                if len(self.detected_cylinders) > 0:
                    selected = self.detected_cylinders[self.selected_cylinder_idx]
                    info = f"Selected: Cylinder {selected['id']}"
                    cv2.putText(display_frame, info, 
                               (display_frame.shape[1] - 300, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # 状态信息
                status_y = 100
                if self.auto_mode:
                    status = f"MODE: AUTO | Queue: {self.execution_queue}"
                    cv2.putText(display_frame, status,
                               (display_frame.shape[1] - 400, status_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 180, 255), 2)
                
                # ==================== 显示 ====================
                cv2.imshow(window_name, display_frame)
                
                # ==================== 键盘输入 ====================
                key = cv2.waitKey(50) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == 82 or key == ord('w'):  # UP arrow or W
                    self.selected_cylinder_idx = max(0, self.selected_cylinder_idx - 1)
                    self.publish_selected_target()
                elif key == 84 or key == ord('s'):  # DOWN arrow or S
                    self.selected_cylinder_idx = min(len(self.detected_cylinders) - 1, self.selected_cylinder_idx + 1)
                    self.publish_selected_target()
                elif key == 13:  # ENTER
                    if len(self.detected_cylinders) > 0:
                        selected_id = self.detected_cylinders[self.selected_cylinder_idx]['id']
                        self.get_logger().info(f"✓ 目标确认: 圆柱 {selected_id}")
                elif key == ord('a'):  # Toggle Auto Mode
                    self.auto_mode = not self.auto_mode
                    if self.auto_mode:
                        self.execution_queue = [cyl['id'] for cyl in self.detected_cylinders]
                        self.get_logger().info(f"✓ 自动模式启用, 队列: {self.execution_queue}")
                    else:
                        self.execution_queue = []
                        self.get_logger().info("✓ 手动模式启用")
            
            except Exception as e:
                self.get_logger().error(f"UI循环错误: {e}")
            
            time.sleep(0.01)
        
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = TargetSelector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
