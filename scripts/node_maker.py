#!/usr/bin/python3

import rclpy
import tf_transformations
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node

from robot_simulation.pid import Pid


class node_maker(Node):
    plan_d = [PoseStamped()]
    twists = Twist()

    pid_x = Pid(7.0, 0, 0.2)
    pid_y = Pid(7.0, 0, 0.2)
    pid_w = Pid(7.0, 0, 0.2)
    count = 0
    step_ = 100

    last_position = np.zeros(2)
    last_pos_angl = np.zeros(2)
    last_orientation = np.zeros(4)
    finish = False

    def __init__(self):
        super().__init__('node_maker')
        self.get_logger().info('start node')

        self.pid_x.sp = self.plan_d[self.count].pose.position.x
        self.pid_y.sp = self.plan_d[self.count].pose.position.y

        orientation_q = self.plan_d[self.count].pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        yaw = tf_transformations.euler_from_quaternion(orientation_list)
        self.pid_w.sp = yaw[2]

        # self.pid_w.sp = math.atan2(self.pid_y.sp - self.last_pos_angl[0], self.pid_x.sp - self.last_pos_angl[1])

        self.create_subscription(PoseStamped, '/goal_pose', self.onClick_points, qos_profile=qos_profile_system_default)
        self.create_subscription(Odometry, '/odom', self.onOdom_data, qos_profile=qos_profile_system_default)
        self.create_subscription(Path, '/plan', self.onPlan, qos_profile=qos_profile_system_default)
        self.plan__publisher = self.create_publisher(Path, '/plan', qos_profile=qos_profile_system_default)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile=qos_profile_system_default)

    def onPlan(self, msg: Path):
        self.plan_d += msg.poses

    def onOdom_data(self, msg: Odometry):
        self.pid_x.pos = msg.pose.pose.position.x
        self.pid_y.pos = msg.pose.pose.position.y

        orientation_q_raw = msg.pose.pose.orientation
        orientation_list_raw = [orientation_q_raw.x, orientation_q_raw.y, orientation_q_raw.z, orientation_q_raw.w]
        yaw_raw = tf_transformations.euler_from_quaternion(orientation_list_raw)
        yaw_raw_ = (yaw_raw[2] + math.pi) % (2 * math.pi) - math.pi
        self.pid_w.pos = yaw_raw_

        if self.finish is not True:
            if self.pid_w.pos < (math.pi/2) or self.pid_w.pos > -(math.pi/2):
                self.twists.linear.x = self.pid_x.pid()
                self.twists.linear.y = self.pid_y.pid()
            else:
                self.twists.linear.x = -self.pid_x.pid()
                self.twists.linear.y = -self.pid_y.pid()
            self.twists.angular.z = self.pid_w.pid()
            # self.twist_publisher.publish(self.twists)

            data_x = self.pid_x.sp - self.pid_x.pos
            data_y = self.pid_y.sp - self.pid_y.pos
            data_z = self.pid_w.sp - self.pid_w.pos

            # print(self.pid_w.sp - self.pid_w.pos)
            print(str(self.pid_w.pos)+" "+str(self.pid_w.sp))
            # print(str(data_x) + " " + str(data_y) + " " + str(data_z))

        if abs(self.pid_x.sp - self.pid_x.pos) < 0.015 and abs(self.pid_y.sp - self.pid_y.pos) < 0.015 :
            if self.count < self.plan_d.__len__():
                self.pid_x.reset_err()
                self.pid_y.reset_err()
                self.pid_w.reset_err()

                self.pid_x.sp = self.plan_d[self.count].pose.position.x
                self.pid_y.sp = self.plan_d[self.count].pose.position.y

                orientation_q = self.plan_d[self.count].pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                yaw = tf_transformations.euler_from_quaternion(orientation_list)
                yaw_r = (yaw[2] + math.pi) % (2 * math.pi) - math.pi
                self.pid_w.sp = yaw_r
                # print(yaw[2])

                # self.pid_w.sp = math.atan2(self.pid_y.sp - self.last_pos_angl[0], self.pid_x.sp - self.last_pos_angl[1])

                self.count += 1
                self.finish = False
            else:
                self.finish = True
                self.last_pos_angl[1] = msg.pose.pose.position.x
                self.last_pos_angl[0] = msg.pose.pose.position.y
                self.twists.linear.x = 0.0
                self.twists.linear.y = 0.0
                self.twists.angular.z = 0.0

        self.twist_publisher.publish(self.twists)

    def onClick_points(self, msg: PoseStamped):
        data_send = Path()
        data_send.header.frame_id = 'odom'
        for i in range(1, self.step_ + 1):
            current_data_x = msg.pose.position.x - self.last_position[0]
            current_data_y = msg.pose.position.y - self.last_position[1]

            current_data_rot_x = msg.pose.orientation.x 
            current_data_rot_y = msg.pose.orientation.y 
            current_data_rot_z = msg.pose.orientation.z 
            current_data_rot_w = msg.pose.orientation.w 

            data = PoseStamped()
            data.pose.position.x = (i * current_data_x / self.step_) + self.last_position[0]
            data.pose.position.y = (i * current_data_y / self.step_) + self.last_position[1]

            data.pose.orientation.x = (i * current_data_rot_x ) 
            data.pose.orientation.y = (i * current_data_rot_y ) 
            data.pose.orientation.z = (i * current_data_rot_z ) 
            data.pose.orientation.w = (i * current_data_rot_w ) 

            data_send.poses.append(data)

        self.plan__publisher.publish(data_send)

        self.last_position[0] = msg.pose.position.x
        self.last_position[1] = msg.pose.position.y
        self.last_orientation[0] = msg.pose.orientation.x
        self.last_orientation[1] = msg.pose.orientation.y
        self.last_orientation[2] = msg.pose.orientation.z
        self.last_orientation[3] = msg.pose.orientation.w


def main(args=None):
    rclpy.init(args=args)
    maker = node_maker()
    try:
        rclpy.spin(maker)
    except KeyboardInterrupt:
        print('Stopped by keyboard interrupt')
        pass
    except BaseException:
        print('Stopped by exception')
        raise
    finally:
        maker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
