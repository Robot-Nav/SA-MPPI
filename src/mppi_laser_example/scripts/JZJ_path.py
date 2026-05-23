#!/usr/bin/env python3
# scripts/fixed_path_publisher.py (ROS1 version)

import rospy
import math
import sys
import select
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# 手动实现四元数与欧拉角转换
def euler_from_quaternion(quat):
    x, y, z, w = quat
    # roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    # yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

class FixedPathPublisher:
    def __init__(self):
        # 参数声明
        self.path_type = rospy.get_param('~path_type', 'straight')
        self.path_length = rospy.get_param('~path_length', 8.0)   # 仅用于直线/矩形
        self.path_points = rospy.get_param('~path_points', 50)
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)
        self.start_x = rospy.get_param('~start_x', 0.0)
        self.start_y = rospy.get_param('~start_y', 0.0)
        self.start_yaw = rospy.get_param('~start_yaw', 0.0)

        # 发布器 (latch=True 使后连接节点也能收到最新路径)
        self.path_pub = rospy.Publisher('plan', Path, queue_size=1, latch=True)

        # 订阅initialpose以手动设置起点（可选）
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.initial_pose_callback)

        # 定时发布
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_path)

        # 生成初始路径
        self.current_path = self.generate_path()

        rospy.loginfo('Fixed Path Publisher initialized')
        rospy.loginfo('Path type: %s', self.path_type)
        rospy.loginfo('Fixed start position: (%.2f, %.2f, %.2f)',
                      self.start_x, self.start_y, self.start_yaw)

        # 立即发布一次
        self.publish_path(None)

    def initial_pose_callback(self, msg):
        """通过initialpose话题设置新的固定起点"""
        self.start_x = msg.pose.pose.position.x
        self.start_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.start_yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        rospy.loginfo('Set new fixed start pose: (%.2f, %.2f, %.2f)',
                      self.start_x, self.start_y, self.start_yaw)
        self.current_path = self.generate_path()

    def generate_path(self):
        """根据类型生成路径"""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.frame_id

        if self.path_type == 'straight':
            poses = self.generate_straight_path()
        elif self.path_type == 'three_side_rectangle':
            poses = self.generate_three_side_rectangle_path()
        elif self.path_type == 'arc_line':          # 新增路径类型
            poses = self.generate_arc_line_path()
        else:
            rospy.logwarn('Unknown path type: %s, using straight', self.path_type)
            poses = self.generate_straight_path()

        path_msg.poses = poses
        return path_msg

    def generate_straight_path(self):
        """直线路径（保留原功能）"""
        poses = []
        end_x = self.start_x + self.path_length * math.cos(self.start_yaw)
        end_y = self.start_y + self.path_length * math.sin(self.start_yaw)

        for i in range(self.path_points):
            t = i / (self.path_points - 1)
            x = self.start_x * (1 - t) + end_x * t
            y = self.start_y * (1 - t) + end_y * t
            yaw = self.start_yaw
            poses.append(self.create_pose_stamped(x, y, yaw))

        rospy.loginfo('Generated straight path from (%.2f, %.2f) to (%.2f, %.2f)',
                      self.start_x, self.start_y, end_x, end_y)
        return poses

    def generate_three_side_rectangle_path(self):
        """4x6m 三边矩形（保留原功能）"""
        poses = []
        width, height = 4.0, 6.0
        segments = [
            {'start': (0.0, 0.0), 'end': (0.0, height)},
            {'start': (0.0, height), 'end': (-width, height)},
            {'start': (-width, height), 'end': (-width, 0.0)}
        ]
        total_len = height + width + height
        pts_side1 = max(2, int(self.path_points * height / total_len))
        pts_side2 = max(2, int(self.path_points * width / total_len))
        pts_side3 = max(2, self.path_points - pts_side1 - pts_side2)
        pts_per_seg = [pts_side1, pts_side2, pts_side3]

        for seg_idx, seg in enumerate(segments):
            start_rel, end_rel = seg['start'], seg['end']
            n = pts_per_seg[seg_idx]
            for i in range(n):
                t = i / (n - 1)
                rel_x = start_rel[0] * (1 - t) + end_rel[0] * t
                rel_y = start_rel[1] * (1 - t) + end_rel[1] * t
                x = self.start_x + rel_x * math.cos(self.start_yaw) - rel_y * math.sin(self.start_yaw)
                y = self.start_y + rel_x * math.sin(self.start_yaw) + rel_y * math.cos(self.start_yaw)

                # 计算方向
                if i < n - 1:
                    nt = (i + 1) / (n - 1)
                    nrel_x = start_rel[0] * (1 - nt) + end_rel[0] * nt
                    nrel_y = start_rel[1] * (1 - nt) + end_rel[1] * nt
                    dx, dy = nrel_x - rel_x, nrel_y - rel_y
                    yaw_rel = math.atan2(dy, dx) if abs(dx) > 1e-6 or abs(dy) > 1e-6 else 0.0
                else:
                    dx, dy = end_rel[0] - start_rel[0], end_rel[1] - start_rel[1]
                    yaw_rel = math.atan2(dy, dx) if abs(dx) > 1e-6 or abs(dy) > 1e-6 else 0.0

                yaw_global = yaw_rel + self.start_yaw
                poses.append(self.create_pose_stamped(x, y, yaw_global))

        rospy.loginfo('Generated three-side rectangle path')
        return poses

    def generate_arc_line_path(self):
        """
        生成圆弧-直线路径（替代原正弦曲线）
        相对坐标系下：起点(0,0) -> 逆时针圆弧 -> 直线 -> 顺时针圆弧 -> 直线 -> 终点(16.8, 3.7)
        圆弧半径 R = 0.5
        """
        poses = []
        # 固定关键点（相对坐标）
        x0, y0 = 0.0, 0.0
        xm, ym = 5.5, 3.7
        xg, yg = 16.8, 3.7
        R = 0.5

        # 计算中间参数
        theta1 = math.atan2(ym - y0, xm - x0)          # 第一段直线方向
        d = R * math.tan(theta1 / 2.0)

        cos_th = math.cos(theta1)
        sin_th = math.sin(theta1)

        # 第一段圆弧相关点
        O1x, O1y = x0, y0 + R
        T1x = O1x + R * sin_th
        T1y = O1y - R * cos_th

        # 第二段直线切点
        T2x = xm - d * cos_th
        T2y = ym - d * sin_th
        T3x = xm + d
        T3y = ym

        # 第二段圆弧圆心
        O2x = T2x + R * sin_th
        O2y = T2y - R * cos_th

        # 计算各段长度
        arc1_len = R * theta1
        line1_len = math.hypot(T2x - T1x, T2y - T1y)
        arc2_len = R * theta1                     # 转弯角度相同
        line2_len = xg - T3x
        total_len = arc1_len + line1_len + arc2_len + line2_len

        # 按长度分配点数（至少2点每段）
        pts_arc1 = max(2, int(self.path_points * arc1_len / total_len))
        pts_line1 = max(2, int(self.path_points * line1_len / total_len))
        pts_arc2 = max(2, int(self.path_points * arc2_len / total_len))
        pts_line2 = max(2, self.path_points - pts_arc1 - pts_line1 - pts_arc2)

        # ---------- 第一段圆弧（逆时针） ----------
        start_angle1 = math.atan2(y0 - O1y, x0 - O1x)          # -pi/2
        end_angle1 = math.atan2(T1y - O1y, T1x - O1x)          # theta1 - pi/2
        if end_angle1 < start_angle1:
            end_angle1 += 2 * math.pi

        for i in range(pts_arc1):
            t = i / (pts_arc1 - 1)
            angle = start_angle1 + t * (end_angle1 - start_angle1)
            x_rel = O1x + R * math.cos(angle)
            y_rel = O1y + R * math.sin(angle)
            # 切线方向（逆时针：径向角 + pi/2）
            yaw_rel = angle + math.pi / 2
            # 变换到全局坐标
            x_global = self.start_x + x_rel * math.cos(self.start_yaw) - y_rel * math.sin(self.start_yaw)
            y_global = self.start_y + x_rel * math.sin(self.start_yaw) + y_rel * math.cos(self.start_yaw)
            yaw_global = yaw_rel + self.start_yaw
            poses.append(self.create_pose_stamped(x_global, y_global, yaw_global))

        # ---------- 第一段直线 (T1 -> T2) ----------
        for i in range(1, pts_line1):          # 从1开始避免重复T1
            t = i / (pts_line1 - 1)
            x_rel = T1x * (1 - t) + T2x * t
            y_rel = T1y * (1 - t) + T2y * t
            yaw_rel = theta1
            x_global = self.start_x + x_rel * math.cos(self.start_yaw) - y_rel * math.sin(self.start_yaw)
            y_global = self.start_y + x_rel * math.sin(self.start_yaw) + y_rel * math.cos(self.start_yaw)
            yaw_global = yaw_rel + self.start_yaw
            poses.append(self.create_pose_stamped(x_global, y_global, yaw_global))

        # ---------- 第二段圆弧（顺时针） ----------
        start_angle2 = math.atan2(T2y - O2y, T2x - O2x)
        end_angle2 = math.atan2(T3y - O2y, T3x - O2x)
        if end_angle2 > start_angle2:
            end_angle2 -= 2 * math.pi

        for i in range(1, pts_arc2):           # 从1开始避免重复T2
            t = i / (pts_arc2 - 1)
            angle = start_angle2 - t * (start_angle2 - end_angle2)   # 递减
            x_rel = O2x + R * math.cos(angle)
            y_rel = O2y + R * math.sin(angle)
            # 切线方向（顺时针：径向角 - pi/2）
            yaw_rel = angle - math.pi / 2
            x_global = self.start_x + x_rel * math.cos(self.start_yaw) - y_rel * math.sin(self.start_yaw)
            y_global = self.start_y + x_rel * math.sin(self.start_yaw) + y_rel * math.cos(self.start_yaw)
            yaw_global = yaw_rel + self.start_yaw
            poses.append(self.create_pose_stamped(x_global, y_global, yaw_global))

        # ---------- 第二段直线 (T3 -> 终点) ----------
        for i in range(1, pts_line2):          # 从1开始避免重复T3
            t = i / (pts_line2 - 1)
            x_rel = T3x * (1 - t) + xg * t
            y_rel = T3y * (1 - t) + yg * t
            yaw_rel = 0.0                       # 水平向右
            x_global = self.start_x + x_rel * math.cos(self.start_yaw) - y_rel * math.sin(self.start_yaw)
            y_global = self.start_y + x_rel * math.sin(self.start_yaw) + y_rel * math.cos(self.start_yaw)
            yaw_global = yaw_rel + self.start_yaw
            poses.append(self.create_pose_stamped(x_global, y_global, yaw_global))

        rospy.loginfo('Generated arc-line path with %d points', len(poses))
        return poses

    def create_pose_stamped(self, x, y, yaw):
        """辅助函数：创建带姿态的PoseStamped"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        # 四元数 from yaw
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def publish_path(self, event):
        """定时发布路径"""
        if self.current_path and len(self.current_path.poses) > 0:
            self.current_path.header.stamp = rospy.Time.now()
            self.path_pub.publish(self.current_path)
            rospy.logdebug('Published path with %d points', len(self.current_path.poses))

    def update_path_type(self, path_type):
        """动态切换路径类型"""
        self.path_type = path_type
        self.current_path = self.generate_path()
        rospy.loginfo('Path type updated to: %s', path_type)


def main():
    rospy.init_node('fixed_path_publisher')
    node = FixedPathPublisher()

    # 命令行交互（非阻塞输入）
    print("\n" + "=" * 50)
    print("Fixed Path Publisher Running (ROS1)")
    print("=" * 50)
    print("Commands:")
    print("  s - switch to straight path")
    print("  t - switch to 4x6m three-side rectangle path")
    print("  a - switch to arc-line path (new)")
    print("  p - print path info")
    print("  Ctrl+C - exit")
    print("=" * 50)
    print("Fixed start: (%.2f, %.2f, %.2f)" % (node.start_x, node.start_y, node.start_yaw))
    print("=" * 50)

    rate = rospy.Rate(10)  # 10 Hz 检查输入
    while not rospy.is_shutdown():
        # 检查标准输入（非阻塞）
        if select.select([sys.stdin], [], [], 0)[0]:
            cmd = sys.stdin.readline().strip().lower()
            if cmd == 's':
                node.update_path_type('straight')
            elif cmd == 't':
                node.update_path_type('three_side_rectangle')
            elif cmd == 'a':
                node.update_path_type('arc_line')
            elif cmd == 'p':
                path = node.current_path
                print("\nPath Info:")
                print("  Type: %s" % node.path_type)
                print("  Fixed start: (%.2f, %.2f, %.2f)" % (node.start_x, node.start_y, node.start_yaw))
                print("  Points: %d" % len(path.poses))
                if len(path.poses) > 0:
                    first = path.poses[0].pose.position
                    last = path.poses[-1].pose.position
                    print("  Start: (%.2f, %.2f)" % (first.x, first.y))
                    print("  End:   (%.2f, %.2f)" % (last.x, last.y))
            else:
                print("Unknown command")
        rate.sleep()

if __name__ == '__main__':
    main()