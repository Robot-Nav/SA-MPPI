#!/usr/bin/env python3
# scripts/mppi_path_publisher.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import numpy as np
from typing import List, Tuple

# 手动实现euler_from_quaternion
def euler_from_quaternion(quat):
    """
    将四元数转换为欧拉角（roll, pitch, yaw）
    quat: [x, y, z, w]
    """
    x, y, z, w = quat
    
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

class FixedPathPublisher(Node):
    def __init__(self):
        super().__init__('fixed_path_publisher')
        
        # 参数声明
        self.declare_parameter('path_type', 'straight')
        self.declare_parameter('path_length', 8.0)
        self.declare_parameter('path_points', 50)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('start_x', 0.0)      # 固定起点x
        self.declare_parameter('start_y', 0.0)      # 固定起点y
        self.declare_parameter('start_yaw', 0.0)    # 固定起点朝向
        
        # 获取参数
        self.path_type = self.get_parameter('path_type').value
        self.path_length = self.get_parameter('path_length').value
        self.path_points = self.get_parameter('path_points').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # 固定路径起点（从参数获取，不随odom更新）
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_yaw = self.get_parameter('start_yaw').value
        
        # 创建发布者
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_pub = self.create_publisher(Path, 'plan', qos)
        
        # 可选：订阅initialpose来手动设置起点（不自动更新）
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.initial_pose_callback,
            10
        )
        
        # 定时器
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_path)
        
        # 生成初始路径
        self.current_path = self.generate_path()
        
        self.get_logger().info('Fixed Path Publisher initialized')
        self.get_logger().info(f'Path type: {self.path_type}')
        self.get_logger().info(f'Fixed start position: ({self.start_x:.2f}, {self.start_y:.2f}, {self.start_yaw:.2f})')
        
        # 立即发布一次
        self.publish_path()
    
    def initial_pose_callback(self, msg):
        """接收初始位姿设置（可选，手动设置新起点）"""
        self.start_x = msg.pose.pose.position.x
        self.start_y = msg.pose.pose.position.y
        
        orientation = msg.pose.pose.orientation
        _, _, self.start_yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        self.get_logger().info(f'Set new fixed start pose: ({self.start_x:.2f}, {self.start_y:.2f}, {self.start_yaw:.2f})')
        self.current_path = self.generate_path()
    
    def generate_path(self) -> Path:
        """根据类型生成固定全局路径"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id
        
        if self.path_type == 'straight':
            poses = self.generate_straight_path()
        elif self.path_type == 'three_side_rectangle':
            poses = self.generate_three_side_rectangle_path()
        elif self.path_type == 'custom':
            poses = self.generate_custom_path()
        else:
            self.get_logger().warn(f'Unknown path type: {self.path_type}, using straight')
            poses = self.generate_straight_path()
        
        path_msg.poses = poses
        return path_msg
    
    def generate_straight_path(self) -> List[PoseStamped]:
        """生成直线路径 - 使用固定起点"""
        poses = []
        
        # 终点坐标（沿固定朝向方向）
        end_x = self.start_x + self.path_length * math.cos(self.start_yaw)
        end_y = self.start_y + self.path_length * math.sin(self.start_yaw)
        
        for i in range(self.path_points):
            t = i / (self.path_points - 1)
            
            # 线性插值
            x = self.start_x * (1 - t) + end_x * t
            y = self.start_y * (1 - t) + end_y * t
            yaw = self.start_yaw
            
            pose = self.create_pose_stamped(x, y, yaw)
            poses.append(pose)
        
        self.get_logger().info(f'Generated straight path from ({self.start_x:.2f}, {self.start_y:.2f}) to ({end_x:.2f}, {end_y:.2f})')
        return poses
    
    def generate_three_side_rectangle_path(self) -> List[PoseStamped]:
        """生成4x6m不闭合的三边矩形路径 - 使用固定起点
           路径形状：起点在右下角，向上走6m，向左走4m，向下走6m
           终点在左上角，起点和终点不重合"""
        poses = []
        
        # 矩形尺寸
        width = 4.0   # x方向宽度（水平方向）
        height = 6.0  # y方向高度（垂直方向）
        
        # 定义三个边的关键点（相对于起点，逆时针方向）
        # 起点：右下角 (0, 0)
        # 第1边：向上到右上角 (0, height)
        # 第2边：向左到左上角 (-width, height)
        # 第3边：向下到左下角 (-width, 0)
        
        # 三个线段
        segments = [
            # 第1边：从起点(0,0)到右上角(0, height)
            {'start': (0.0, 0.0), 'end': (0.0, height)},
            
            # 第2边：从右上角(0, height)到左上角(-width, height)
            {'start': (0.0, height), 'end': (-width, height)},
            
            # 第3边：从左上角(-width, height)到左下角(-width, 0)
            {'start': (-width, height), 'end': (-width, 0)}
        ]
        
        # 计算每个边的点数（按路径长度比例分配）
        total_length = height + width + height  # 6 + 4 + 6 = 16m
        points_side1 = max(2, int(self.path_points * height / total_length))
        points_side2 = max(2, int(self.path_points * width / total_length))
        points_side3 = max(2, self.path_points - points_side1 - points_side2)
        
        points_per_segment = [points_side1, points_side2, points_side3]
        
        # 生成每个边的点
        for seg_idx, segment in enumerate(segments):
            start_rel = segment['start']
            end_rel = segment['end']
            n_points = points_per_segment[seg_idx]
            
            for i in range(n_points):
                t = i / (n_points - 1)
                
                # 相对坐标
                rel_x = start_rel[0] * (1 - t) + end_rel[0] * t
                rel_y = start_rel[1] * (1 - t) + end_rel[1] * t
                
                # 转换为全局坐标（考虑起点朝向）
                x = self.start_x + rel_x * math.cos(self.start_yaw) - rel_y * math.sin(self.start_yaw)
                y = self.start_y + rel_x * math.sin(self.start_yaw) + rel_y * math.cos(self.start_yaw)
                
                # 计算朝向（沿路径方向）
                if i < n_points - 1:
                    next_t = (i + 1) / (n_points - 1)
                    next_rel_x = start_rel[0] * (1 - next_t) + end_rel[0] * next_t
                    next_rel_y = start_rel[1] * (1 - next_t) + end_rel[1] * next_t
                    dx = next_rel_x - rel_x
                    dy = next_rel_y - rel_y
                    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                        yaw = self.start_yaw
                    else:
                        yaw = math.atan2(dy, dx) + self.start_yaw
                else:
                    # 最后一个点使用当前边的方向
                    dx = end_rel[0] - start_rel[0]
                    dy = end_rel[1] - start_rel[1]
                    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                        yaw = self.start_yaw
                    else:
                        yaw = math.atan2(dy, dx) + self.start_yaw
                
                pose = self.create_pose_stamped(x, y, yaw)
                poses.append(pose)
        
        # 计算实际终点坐标（左下角）
        end_rel_x = -width
        end_rel_y = 0.0
        end_x = self.start_x + end_rel_x * math.cos(self.start_yaw) - end_rel_y * math.sin(self.start_yaw)
        end_y = self.start_y + end_rel_x * math.sin(self.start_yaw) + end_rel_y * math.cos(self.start_yaw)
        
        self.get_logger().info(f'Generated 4x6m three-side rectangle path')
        self.get_logger().info(f'  Start point (右下角): ({self.start_x:.2f}, {self.start_y:.2f})')
        self.get_logger().info(f'  End point (左下角): ({end_x:.2f}, {end_y:.2f})')
        self.get_logger().info(f'  Points per side: {points_side1}, {points_side2}, {points_side3}')
        
        return poses
    
    def generate_custom_path(self) -> List[PoseStamped]:
        """生成自定义S形路径 - 使用固定起点"""
        poses = []
        
        for i in range(self.path_points):
            t = i / (self.path_points - 1)
            
            # 相对坐标
            rel_x = self.path_length * t
            rel_y = 0.5 * math.sin(2 * math.pi * t)
            
            # 转换为全局坐标
            x = self.start_x + rel_x * math.cos(self.start_yaw) - rel_y * math.sin(self.start_yaw)
            y = self.start_y + rel_x * math.sin(self.start_yaw) + rel_y * math.cos(self.start_yaw)
            
            # 计算切线方向
            if i < self.path_points - 1:
                next_t = (i + 1) / (self.path_points - 1)
                next_rel_x = self.path_length * next_t
                next_rel_y = 0.5 * math.sin(2 * math.pi * next_t)
                dx = next_rel_x - rel_x
                dy = next_rel_y - rel_y
                yaw = math.atan2(dy, dx) + self.start_yaw
            else:
                yaw = self.start_yaw
            
            pose = self.create_pose_stamped(x, y, yaw)
            poses.append(pose)
        
        self.get_logger().info('Generated custom S-shaped path')
        return poses
    
    def create_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        """创建带姿态的PoseStamped消息"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose
    
    def publish_path(self):
        """发布当前路径"""
        if self.current_path and len(self.current_path.poses) > 0:
            self.current_path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.current_path)
            self.get_logger().debug(f'Published path with {len(self.current_path.poses)} points')
    
    def update_path_type(self, path_type: str):
        """动态更新路径类型"""
        self.path_type = path_type
        self.current_path = self.generate_path()
        self.get_logger().info(f'Path type updated to: {path_type}')


def main(args=None):
    rclpy.init(args=args)
    
    node = FixedPathPublisher()
    
    # 命令行交互
    try:
        print("\n" + "="*50)
        print("Fixed Path Publisher Running")
        print("="*50)
        print("Commands:")
        print("  s - switch to straight path")
        print("  t - switch to 4x6m three-side rectangle path (不闭合)")
        print("  u - switch to custom S-shaped path")
        print("  p - print path info")
        print("  Ctrl+C - exit")
        print("="*50)
        print(f"Fixed start: ({node.start_x:.2f}, {node.start_y:.2f}, {node.start_yaw:.2f})")
        print("="*50)
        
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            import sys
            import select
            if select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.readline().strip().lower()
                if cmd == 's':
                    node.update_path_type('straight')
                elif cmd == 't':
                    node.update_path_type('three_side_rectangle')
                elif cmd == 'u':
                    node.update_path_type('custom')
                elif cmd == 'p':
                    path = node.current_path
                    print(f"\nPath Info:")
                    print(f"  Type: {node.path_type}")
                    print(f"  Fixed start: ({node.start_x:.2f}, {node.start_y:.2f}, {node.start_yaw:.2f})")
                    print(f"  Points: {len(path.poses)}")
                    if len(path.poses) > 0:
                        print(f"  Start: ({path.poses[0].pose.position.x:.2f}, {path.poses[0].pose.position.y:.2f})")
                        print(f"  End: ({path.poses[-1].pose.position.x:.2f}, {path.poses[-1].pose.position.y:.2f})")
                
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
