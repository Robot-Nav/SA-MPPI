#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import math

class CylinderMover:
    def __init__(self):
        rospy.init_node('path_following_cylinder', anonymous=True)

        # Gazebo 服务初始化
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # 获取模型状态的服务
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # 初始位置和姿态
        self.x = 14.0  # X 初始位置
        self.y = -0.55   # Y 初始位置
        self.z = 0.17   # Z 高度 (固定)
        self.yaw = -1.57  # 偏航角 (弧度)
        
        # 障碍物运动参数
        self.max_speed = 0.30       # 最大速度 [m/s]
        self.acceleration = 0.10    # 加速度 [m/s^2]
        self.linear_velocity = 0.0  # 当前速度 [m/s]

        # 预设路径 (一系列的 x, y 点)
        self.path = [
            (14.0, 0.00),
            (14.0, 2.0),
            (15.0, 3.0),
            (17.0, 3.65),
            (19.0, 3.65),
            (40.0, 3.65)
        ]
        self.current_target_idx = 0  # 当前目标路径点索引

        # 运动标志 - 当检测到mobile_base到达x=5时设为True
        self.start_moving = False
        
        # 时间
        self.last_time = rospy.Time.now()

        # 循环频率
        self.rate = rospy.Rate(20)  # 20Hz 刷新率

        # 开始主循环
        self.run()

    def check_robot_position(self):
        """
        检查mobile_base机器人是否到达x=5位置
        """
        try:
            # 获取mobile_base模型状态
            model_state = self.get_model_state("mobile_base", "world")
            
            if model_state.success:
                robot_x = model_state.pose.position.x
                rospy.loginfo("mobile_base position: x=%.2f", robot_x)
                
                # 当机器人x坐标大于等于5时启动
                if robot_x >= 5.5 and not self.start_moving:
                    rospy.loginfo("mobile_base reached x=5.0! Starting cylinder movement...")
                    self.start_moving = True
                    self.last_time = rospy.Time.now()  # 重置时间，确保初始dt计算正确
            else:
                rospy.logwarn("Failed to get mobile_base state")
                
        except rospy.ServiceException as e:
            rospy.logerr("Failed to get model state: %s" % e)

    def distance_to_target(self, target_x, target_y):
        """
        计算当前位置到目标点的欧式距离
        """
        return math.sqrt((target_x - self.x)**2 + (target_y - self.y)**2)

    def update_position(self, dt):
        """
        根据路径点更新位置和朝向
        """
        # 获取当前目标点
        target_x, target_y = self.path[self.current_target_idx]

        # 计算到目标点的方向角
        angle_to_target = math.atan2(target_y - self.y, target_x - self.x)
        
        # 匀加速逻辑
        if self.linear_velocity < self.max_speed:
            self.linear_velocity += self.acceleration * dt
        else:
            self.linear_velocity = self.max_speed

        # 根据线速度和方向角更新位置
        self.x += self.linear_velocity * math.cos(angle_to_target) * dt
        self.y += self.linear_velocity * math.sin(angle_to_target) * dt
        self.z = 0.17
        self.yaw = angle_to_target  # 姿态朝向目标方向

        # 判断是否接近目标点
        if self.distance_to_target(target_x, target_y) < 0.1:  # 距离阈值
            rospy.loginfo("Reached target point: (%.2f, %.2f)", target_x, target_y)
            self.current_target_idx += 1  # 切换到下一个路径点

            # 如果已经到达路径终点，停止
            if self.current_target_idx >= len(self.path):
                rospy.loginfo("Path completed!")
                rospy.signal_shutdown("Path completed")

    def run(self):
        """
        主循环：检查机器人位置，更新圆柱体位置并调用 Gazebo 服务刷新状态
        """
        while not rospy.is_shutdown():
            # 先检查mobile_base位置
            self.check_robot_position()
            
            if self.start_moving:
                # 只有在mobile_base到达x=5后才开始移动圆柱体
                
                # 计算时间差
                current_time = rospy.Time.now()
                dt = (current_time - self.last_time).to_sec()
                self.last_time = current_time
                
                # 防止dt过大导致跳跃
                if dt < 0.1:  # 如果dt小于0.1秒才更新位置，避免初始dt过大
                    self.update_position(dt)
                
                    # 构建 Gazebo 的 SetModelState 请求
                    model_state_request = SetModelStateRequest()
                    model_state_request.model_state.model_name = "unit_cylinder"  # 模型名称
                    model_state_request.model_state.pose.position.x = self.x
                    model_state_request.model_state.pose.position.y = self.y
                    model_state_request.model_state.pose.position.z = self.z

                    # 设置朝向（四元数）
                    qz = math.sin((self.yaw - 1.57) / 2.0)
                    qw = math.cos((self.yaw - 1.57) / 2.0)
                    model_state_request.model_state.pose.orientation.z = qz
                    model_state_request.model_state.pose.orientation.w = qw

                    model_state_request.model_state.reference_frame = "world"

                    try:
                        self.set_model_state(model_state_request)
                        rospy.loginfo("Moving to: x=%.2f, y=%.2f, yaw=%.2f", self.x, self.y, self.yaw)
                    except rospy.ServiceException as e:
                        rospy.logerr("Failed to update model state: %s" % e)
            else:
                # 如果还没启动，保持圆柱体在初始位置
                rospy.loginfo("Waiting for mobile_base to reach x=5.0... Current cylinder position: (%.2f, %.2f)", self.x, self.y)
                
                # 仍然需要更新圆柱体在Gazebo中的初始位置
                model_state_request = SetModelStateRequest()
                model_state_request.model_state.model_name = "unit_cylinder"
                model_state_request.model_state.pose.position.x = self.x
                model_state_request.model_state.pose.position.y = self.y
                model_state_request.model_state.pose.position.z = self.z
                
                qz = math.sin((self.yaw - 1.57) / 2.0)
                qw = math.cos((self.yaw - 1.57) / 2.0)
                model_state_request.model_state.pose.orientation.z = qz
                model_state_request.model_state.pose.orientation.w = qw
                model_state_request.model_state.reference_frame = "world"
                
                try:
                    self.set_model_state(model_state_request)
                except rospy.ServiceException as e:
                    rospy.logerr("Failed to update model state: %s" % e)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        CylinderMover()
    except rospy.ROSInterruptException:
        pass
