#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand

class WaypointSender:
    def __init__(self):
        rospy.init_node('waypoint_sender', anonymous=True)

        # 从参数服务器加载航点
        self.waypoints = rospy.get_param('~waypoints', [])
        if not self.waypoints:
            rospy.logerr("No waypoints found in parameter server. Shutting down.")
            rospy.signal_shutdown("No waypoints")
            return

        rospy.loginfo("Loaded %d waypoints.", len(self.waypoints))

        # 初始化状态变量
        self.current_waypoint_index = 0
        self.current_position = None
        self.odom_received = False
        self.distance_threshold = 0.3  # 到达航点的距离阈值, 单位: 米

        # ROS 通信接口
        # 订阅无人机里程计
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        # 发布 PositionCommand 指令给 main_ctrl
        self.cmd_pub = rospy.Publisher("position_cmd", PositionCommand, queue_size=1)

        # 创建一个定时器，以10Hz的频率执行主循环
        self.mission_timer = rospy.Timer(rospy.Duration(0.1), self.mission_loop_callback)
        
        rospy.loginfo("Waypoint Sender node initialized. Waiting for odometry...")

    def odom_callback(self, msg):
        """里程计回调函数，更新无人机当前位置"""
        self.current_position = msg.pose.pose.position
        if not self.odom_received:
            self.odom_received = True
            rospy.loginfo("Odometry received. Starting mission.")

    def mission_loop_callback(self, event):
        """任务主循环，由定时器触发"""
        if not self.odom_received:
            # 如果还没有收到里程计信息，则不执行任何操作
            return

        # 检查任务是否已完成
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo_once("Mission complete. Holding final position.")
            # 保持在最后一个航点的位置
            target_waypoint = self.waypoints[-1]
        else:
            target_waypoint = self.waypoints[self.current_waypoint_index]

            # 计算到当前目标航点的距离
            dist = math.sqrt(
                (self.current_position.x - target_waypoint[0])**2 +
                (self.current_position.y - target_waypoint[1])**2 +
                (self.current_position.z - target_waypoint[2])**2
            )

            # 如果距离小于阈值，则认为已到达，并切换到下一个航点
            if dist < self.distance_threshold:
                rospy.loginfo("Waypoint %d reached!", self.current_waypoint_index)
                self.current_waypoint_index += 1
                
                # 立即更新目标点，避免延迟
                if self.current_waypoint_index < len(self.waypoints):
                    target_waypoint = self.waypoints[self.current_waypoint_index]
                    rospy.loginfo("Heading to next waypoint %d: %s", self.current_waypoint_index, str(target_waypoint))
                else:
                    rospy.loginfo("All waypoints reached. Mission complete.")
                    target_waypoint = self.waypoints[-1] # 更新为最后一个点

        # 无论是否切换航点，都持续发布当前的目标指令
        self.publish_position_command(target_waypoint)

    def publish_position_command(self, waypoint):
        """构建并发布 PositionCommand 消息"""
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"  # 假设在世界坐标系下

        # --- 这是与main_ctrl通信的核心 ---
        # 1. 设置期望位置
        cmd.position.x = waypoint[0]
        cmd.position.y = waypoint[1]
        cmd.position.z = waypoint[2]

        # 2. 设置期望速度、加速度为0
        cmd.velocity.x = 0
        cmd.velocity.y = 0
        cmd.velocity.z = 0
        cmd.acceleration.x = 0
        cmd.acceleration.y = 0
        cmd.acceleration.z = 0

        # 3. 计算并设置期望偏航角 (让无人机朝向下一个目标点)
        dx = waypoint[0] - self.current_position.x
        dy = waypoint[1] - self.current_position.y
        # 只有在水平距离大于一个很小的值时才更新偏航角，避免在目标点正上方时发生旋转
        if math.sqrt(dx**2 + dy**2) > 0.1:
            cmd.yaw = math.atan2(dy, dx)
        else:
            cmd.yaw = 0 # 默认朝向
            
        cmd.yaw_dot = 0

        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        ws = WaypointSender()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass