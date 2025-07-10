#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64, UInt8
from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class AudibotController:
    def __init__(self):
        rospy.init_node('audibot_topic_controller_with_feedback')

        # Publishers
        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=10)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=10)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=10)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=10)

        # Subscribers
        rospy.Subscriber('/twist', TwistStamped, self.twist_callback)
        rospy.Subscriber('/steering_state', Float64, self.steering_state_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)  # Lidar aboneliği

        # Current values
        self.current_speed = 0.0
        self.current_steering_angle = 0.0
        self.current_angular_velocity = 0.0
        self.current_yaw = 0.0
        self.initial_yaw = None
        self.steer_fl_angle = 0.0
        self.steer_fr_angle = 0.0

        # Lidar obstacle detection
        self.closest_obstacle_distance = float('inf')
        self.obstacle_distance_threshold = 4.5  # 1 metre mesafe eşiği

        # Fren durumu takibi
        self.is_braking = False

        # Control parameters
        self.desired_yaw = math.pi / 2  # 90 degrees

        # Başlangıç komutu: ilerle
        self.command_timeline = [
            [0.2, 0.0, 0.08, 0.0, 0],
        ]

        # Zaman takibi
        self.current_command_index = 0
        self.start_time = rospy.get_time()
        self.current_duration = self.command_timeline[0][0]

        # Komutlar
        self.steering_angle_cmd = self.command_timeline[0][1]
        self.throttle_cmd = self.command_timeline[0][2]
        self.brake_torque_cmd = self.command_timeline[0][3]
        self.gear_cmd = self.command_timeline[0][4]

        self.rate = rospy.Rate(20)  # 20 Hz

    def twist_callback(self, msg):
        self.current_speed = msg.twist.linear.x
        self.current_angular_velocity = msg.twist.angular.z

    def steering_state_callback(self, msg):
        self.current_steering_angle = msg.data

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name == 'steer_fl_joint':
                self.steer_fl_angle = position
            elif name == 'steer_fr_joint':
                self.steer_fr_angle = position

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        if self.initial_yaw is None:
            self.initial_yaw = yaw

        self.current_yaw = yaw - self.initial_yaw
        self.current_yaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))

    def lidar_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if r > 0.0 and not math.isinf(r) and not math.isnan(r)]
        if valid_ranges:
            self.closest_obstacle_distance = min(valid_ranges)
        else:
            self.closest_obstacle_distance = float('inf')

        # Engel mesafesini her lidar mesajında göster (fren modunda değilse)
        if not self.is_braking:
            rospy.loginfo(f"[Lidar] En yakın engel mesafesi: {self.closest_obstacle_distance:.2f} m")

    def update_commands(self):
        current_time = rospy.get_time()
        elapsed_time = current_time - self.start_time

        if elapsed_time >= self.current_duration:
            self.current_command_index = (self.current_command_index + 1) % len(self.command_timeline)
            self.start_time = current_time
            self.current_duration = self.command_timeline[self.current_command_index][0]
            self.steering_angle_cmd = self.command_timeline[self.current_command_index][1]
            self.throttle_cmd = self.command_timeline[self.current_command_index][2]
            self.brake_torque_cmd = self.command_timeline[self.current_command_index][3]
            self.gear_cmd = self.command_timeline[self.current_command_index][4]

        yaw_error = self.desired_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        return yaw_error

    def run(self):
        while not rospy.is_shutdown():
            if self.closest_obstacle_distance < self.obstacle_distance_threshold:
                if not self.is_braking:
                    self.is_braking = True
                    rospy.logwarn(f"Engel tespit edildi, araç durdu! Engel mesafesi: {self.closest_obstacle_distance:.2f} m")
                    rospy.logwarn("FREN YAPILDI!")
                self.command_timeline = [[0.2, 0.0, 0.0, 1.0, 0]]
                self.current_command_index = 0
                self.start_time = rospy.get_time()
                self.current_duration = self.command_timeline[0][0]
            else:
                if self.is_braking:
                    self.is_braking = False
                    rospy.loginfo("Engel kalktı, tekrar ilerleniyor.")
                self.command_timeline = [[0.2, 0.0, 0.08, 0.0, 0]]
                self.current_command_index = 0
                self.start_time = rospy.get_time()
                self.current_duration = self.command_timeline[0][0]

            yaw_error = self.update_commands()

            steering = self.steering_angle_cmd
            throttle = self.throttle_cmd
            brake = self.brake_torque_cmd
            gear = self.gear_cmd

            self.steering_pub.publish(Float64(steering))
            self.throttle_pub.publish(Float64(throttle))
            self.brake_pub.publish(Float64(brake))
            self.gear_pub.publish(UInt8(gear))

            # Fren modunda sadece fren mesajı, değilse tüm bilgiler
            if not self.is_braking:
                rospy.loginfo(f"Komut {self.current_command_index}: "
                              f"Süre: {self.current_duration}s | "
                              f"Hız: {self.current_speed:.2f} m/s | "
                              f"Direksiyon Cmd: {steering:.3f} rad | "
                              f"FL: {self.steer_fl_angle:.3f} rad | "
                              f"FR: {self.steer_fr_angle:.3f} rad | "
                              f"Yaw: {self.current_yaw:.3f} rad ({self.current_yaw*180/math.pi:.1f} deg) | "
                              f"Yaw Hata: {yaw_error:.3f} rad | "
                              f"Gaz: {throttle:.2f}")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = AudibotController()
        node.run()
    except rospy.ROSInterruptException:
        pass
