#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def compute_2D_distance(coord_1, coord_2):
    """计算x-y平面上两点距离。
    params:
        coord_1 (list): 坐标1
        coord_2 (list): 坐标2
    return:
        float: 两点距离
    """
    distance = math.sqrt(
        (coord_1[0] - coord_2[0]) ** 2 + (coord_1[1] - coord_2[1]) ** 2
    )
    return distance


def compute_angle(angle_src, angle_dst):
    """计算从源角度到目标角度的逆时针旋转角度。
    params:
        angle_src (float): 源角度, [-180, 180]
        angle_dst (float): 目标角度, [-180, 180]
    return:
        float: 旋转角度, [-180, 180)
    """
    angle = (angle_dst - angle_src + 180) % 360 - 180
    return angle


class RobotController:
    def __init__(self, node_name="robot_controller"):
        rospy.init_node(node_name, anonymous=True)
        # 发布运动指令
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # 订阅传感器消息获取真实位姿
        self.state_sub = rospy.Subscriber(
            "/ground_truth/state", Odometry, self.odometry_callback
        )
        self.pose_6d = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.rate = rospy.Rate(20)

    def odometry_callback(self, msg):
        """获取位姿的回调函数。
        params:
            msg (nav_msgs.msg.Odometry): 真实里程计消息
        """
        orientation = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        self.pose_6d = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                math.degrees(roll),
                math.degrees(pitch),
                math.degrees(yaw),
            ]
        )

    def cmd_stop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def cmd_move_forward(self, speed=1.0, duration=1.0):
        """前后方向运动，指定速度和时间。
        params:
            speed (float): >0前进, <0后退
            duration (float): >0
        """
        twist = Twist()
        twist.linear.x = 1.0 * speed
        rospy.loginfo(f"Robot moving forward {speed} m/s, {duration} s.")
        start_time = rospy.Time.now()

        while (
            not rospy.is_shutdown()
            and (rospy.Time.now() - start_time).to_sec() < duration
        ):
            self.cmd_pub.publish(twist)
            self.rate.sleep()

        self.cmd_stop()
        rospy.loginfo(f"Robot pose: {self.pose_6d}.")

    def cmd_move_forward_distance(self, speed=1.0, distance=1.0):
        """前后方向移动，指定速度和距离。根据真实移动距离判断。
        params:
            speed (float): >0前进, <0后退
            distance (float): >0
        """
        twist = Twist()
        init_coord = self.pose_6d[:2]
        journey = 0.0  # 走过的距离
        rospy.loginfo(
            f"Robot moving forward a distance: {distance if speed > 0 else -distance} m."
        )

        while not rospy.is_shutdown() and abs(journey) < abs(distance):
            if abs(journey - distance) > 0.5:
                twist.linear.x = 1.0 * speed
            else:
                twist.linear.x = 0.4 * speed
            self.cmd_pub.publish(twist)
            self.rate.sleep()
            current_coord = self.pose_6d[:2]
            journey = compute_2D_distance(init_coord, current_coord)

        self.cmd_stop()
        rospy.loginfo(f"Robot pose: {self.pose_6d}.")

    def cmd_turn_left(self, linear_speed=0.1, angular_speed=0.4, duration=1.0):
        """左右转弯，指定线速度、角速度、时间。
        params:
            linear_speed (float): >0前进, <0后退
            angular_speed (float): >0左转, <0右转
            duration (float): >0
        """
        twist = Twist()
        twist.linear.x = 1.0 * linear_speed
        twist.angular.z = 1.0 * angular_speed
        rospy.loginfo(
            f"Robot turning left, linear speed: {linear_speed} m/s, angular speed: {angular_speed} rad/s, {duration} s."
        )
        start_time = rospy.Time.now()

        while (
            not rospy.is_shutdown()
            and (rospy.Time.now() - start_time).to_sec() < duration
        ):
            self.cmd_pub.publish(twist)
            self.rate.sleep()

        self.cmd_stop()
        rospy.loginfo(f"Robot pose: {self.pose_6d}.")

    def cmd_turn_angle(self, angle, linear_speed=0.1, angular_speed=0.4):
        """转过一个角度，指定线速度、角速度。
        params:
            angle (float): 0<__<180向左转, -180<__<0向右转
            linear_speed (float): >0前进, <0后退
            angular_speed (float): >0左转, <0右转
        """
        twist = Twist()
        init_yaw = self.pose_6d[5]
        angle = compute_angle(0, angle)
        angle_turned = 0.0  # 已转过的角度
        rospy.loginfo(f"Robot turning, {angle} degrees.")

        while not rospy.is_shutdown() and abs(angle_turned) < abs(angle):
            if abs(angle - angle_turned) > 10:
                twist.linear.x = 1.0 * linear_speed
                twist.angular.z = (1.0 if angle > 0 else -1.0) * angular_speed
            else:
                twist.linear.x = 0.4 * linear_speed
                twist.angular.z = (0.4 if angle > 0 else -0.4) * angular_speed
            self.cmd_pub.publish(twist)
            self.rate.sleep()
            angle_turned = compute_angle(init_yaw, self.pose_6d[5])

        self.cmd_stop()
        rospy.loginfo(f"Robot pose: {self.pose_6d}")

    # def cmd_turn_to_yaw(self, target_yaw, linear_speed=0.1, angular_speed=0.4):
    #     """转向指定角度
    #     params:
    #         target_yaw (float): >0向左转, <0向右转
    #         linear_speed (float): >0前进, <0后退
    #         angular_speed (float): >0左转, <0右转
    #     """
    #     angle = compute_angle(self.pose_6d[5], target_yaw)
    #     self.cmd_turn_angle(angle, linear_speed, angular_speed)

    def cmd_turn_to_yaw(self, target_yaw, linear_speed=0.1, angular_speed=0.4):
        """转向指定角度，并回到原地。如果转向角大于180，转两次
        params:
            target_yaw (float): >0向左转, <0向右转
            linear_speed (float): >0前进, <0后退
            angular_speed (float): >0左转, <0右转
        """
        rospy.loginfo(f"Robot turning to yaw: {target_yaw} degrees.")
        angle = compute_angle(self.pose_6d[5], target_yaw)
        radius = linear_speed / angular_speed  # 转弯半径
        if abs(angle) <= 90:
            backward_distance = radius * math.tan(math.radians(abs(angle) / 2))
            self.cmd_move_forward_distance(-linear_speed * 2, backward_distance)
            self.cmd_turn_angle(angle, linear_speed, angular_speed)
            # self.cmd_move_forward_distance(-linear_speed * 2, backward_distance)
        else:
            angle = angle / 2
            backward_distance = radius * math.tan(math.radians(abs(angle) / 2))
            self.cmd_move_forward_distance(-linear_speed * 2, backward_distance)
            self.cmd_turn_angle(angle, linear_speed, angular_speed)
            self.cmd_move_forward_distance(-linear_speed * 2, 2 * backward_distance)
            self.cmd_turn_angle(angle, linear_speed, angular_speed)
            # self.cmd_move_forward_distance(-linear_speed * 2, backward_distance)

    def cmd_move_to_position(self, target_coord, linear_speed=0.1, angular_speed=0.4):
        """移动到指定坐标，先转向后前进。
        params:
            target_coord (list): 目标坐标 (x, y)
            linear_speed (float): >0前进, <0后退
            angular_speed (float): >0左转, <0右转
        """
        twist = Twist()
        # 首先转向
        current_theta = math.degrees(
            math.atan2(
                target_coord[1] - self.pose_6d[1], target_coord[0] - self.pose_6d[0]
            )
        )  # 当前目标点与机器人连线等价的偏航角
        current_yaw = self.pose_6d[5]
        if compute_angle(current_yaw, current_theta) > 0:
            # 左转
            while (
                not rospy.is_shutdown()
                and abs(compute_angle(current_yaw, current_theta)) > 0.3
            ):
                if abs(compute_angle(current_yaw, current_theta)) > 5:
                    twist.linear.x = 1.0 * linear_speed
                    twist.angular.z = 1.0 * angular_speed
                else:
                    twist.linear.x = 0.4 * linear_speed
                    twist.angular.z = 0.4 * angular_speed
                self.cmd_pub.publish(twist)
                self.rate.sleep()
                current_theta = math.degrees(
                    math.atan2(
                        target_coord[1] - self.pose_6d[1],
                        target_coord[0] - self.pose_6d[0],
                    )
                )
                current_yaw = self.pose_6d[5]
        else:
            # 右转
            while (
                not rospy.is_shutdown()
                and abs(compute_angle(current_yaw, current_theta)) > 0.3
            ):
                if abs(compute_angle(current_yaw, current_theta)) > 5:
                    twist.linear.x = 1.0 * linear_speed
                    twist.angular.z = -1.0 * angular_speed
                else:
                    twist.linear.x = 0.4 * linear_speed
                    twist.angular.z = -0.4 * angular_speed
                self.cmd_pub.publish(twist)
                self.rate.sleep()
                current_theta = math.degrees(
                    math.atan2(
                        target_coord[1] - self.pose_6d[1],
                        target_coord[0] - self.pose_6d[0],
                    )
                )
                current_yaw = self.pose_6d[5]

        # 转到朝向目标点后前进
        rospy.loginfo(
            f"Robot moving from {self.pose_6d[:2]} to coordinate: {target_coord}."
        )
        distance = compute_2D_distance(self.pose_6d[:2], target_coord)
        self.cmd_move_forward_distance(linear_speed * 2, distance)

    def run(self):
        rospy.loginfo("Robot move start.")
        rospy.sleep(1)
        # 从原点出发
        self.cmd_move_to_position([2.0, -1.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([7.0, -3.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([8.0, -5.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([7.0, -7.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([5.0, -7.5], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([3.8, -6.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([3.5, -3.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([5.0, 1.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([5.0, 6.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([0.0, 8.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([-6.0, 9.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([-7.0, 10.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([-4.0, 10.5], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([-3.0, 7.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([-7.0, 5.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([-7.5, 2.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([-4.0, -2.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([-6.0, -5.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([-1.0, -7.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([0.5, -6.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([0.5, -3.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        self.cmd_move_to_position([0.0, 0.0], 0.3, 0.6)
        # rospy.sleep(0.5)
        rospy.loginfo("Robot move finished.")


if __name__ == "__main__":
    try:
        controller = RobotController("robot_controller")
        controller.run()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"ROS Interrupt Exception: {e}")
