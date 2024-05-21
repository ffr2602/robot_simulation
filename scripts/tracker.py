#!/usr/bin/python3

import rclpy
import tf_transformations
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped, Pose, Quaternion
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
    
    kp = 2.0
    ki = 1.0

    limit = 0.7
    int_x_err = 0.0
    last_pos_angl = np.zeros(2)
    last_position = np.zeros(2)
    finish = False

    def __init__(self):
        super().__init__('node_maker')
        self.get_logger().info('start node')

        self.create_subscription(PoseStamped, '/goal_pose', self.onClick_points, qos_profile=qos_profile_system_default)
        self.create_subscription(Odometry, '/odom', self.onOdom_data, qos_profile=qos_profile_system_default)
        self.create_subscription(Path, '/plan', self.onPlan, qos_profile=qos_profile_system_default)
        self.plan__publisher = self.create_publisher(Path, '/plan', qos_profile=qos_profile_system_default)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile=qos_profile_system_default)

        self.reference_matrix = self.transform_to_matrix(self.pose_to_transform(self.plan_d[self.count].pose))

    def onPlan(self, msg: Path):
        self.plan_d += msg.poses

    def pose_to_transform(self, pose: Pose):
        transform = TransformStamped()
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        return transform

    def transform_to_matrix(self, transform: TransformStamped):
        translation = np.array([transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z])
        rotation = tf_transformations.quaternion_matrix([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        matrix = rotation
        matrix[:3, 3] = translation
        return matrix

    def is_zero(self, matrix, tol):
        return np.allclose(matrix, 0, atol=tol)

    def onOdom_data(self, msg: Odometry):
        current_matrix = self.transform_to_matrix(self.pose_to_transform(msg.pose.pose))        

        global log, log3, x_err

        if self.finish is not True:

            current_matrix_inv = np.linalg.inv(current_matrix)
            tf_diff_matrix = np.dot(current_matrix_inv, self.reference_matrix)
            rot_mat = np.array(tf_diff_matrix[:3, :3])
            trans_vec = tf_diff_matrix[:3, 3]

            acosinput = (np.trace(rot_mat) - 1.0) / 2.0

            if acosinput >= 1.0:
                log3 = [
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0]
                ]
            elif acosinput <= -1.0:
                global omg
                if 1.0 + rot_mat[2, 2] > 0.0001 or -1.0 - rot_mat[2, 2] < -0.0001:
                    vec = [rot_mat[0, 2], rot_mat[1, 2], 1.0 + rot_mat[2, 2]]
                    omg = (1.0 / math.sqrt(2.0 * (1 + rot_mat[2, 2]))) * vec
                elif 1.0 + rot_mat[1, 1] > 0.0001 or -1.0 - rot_mat[1, 1] < -0.0001:
                    vec = [rot_mat[0, 1], 1.0 + rot_mat[1, 1], rot_mat[2, 1]]
                    omg = (1.0 / math.sqrt(2.0 * (1 + rot_mat[1, 1]))) * vec
                else:
                    vec = [1.0 + rot_mat[0, 0], rot_mat[1, 0], rot_mat[2, 0]]
                    omg = (1.0 / math.sqrt(2.0 * (1 + rot_mat[0, 0]))) * vec
                m = math.pi * omg
                log3 = [
                    [0.0, -m[2], m[1]],
                    [m[2], 0.0, -m[0]],
                    [-m[1], m[0], 0.0],
                ]
            else:
                theta = math.acos(acosinput)
                log3 = (theta / 2.0 / math.sin(theta)) * (rot_mat - rot_mat.transpose())

            if self.is_zero(log3, 0.01):
                log = [
                    [0.0, 0.0, 0.0, tf_diff_matrix[0, 3]],
                    [0.0, 0.0, 0.0, tf_diff_matrix[1, 3]],
                    [0.0, 0.0, 0.0, tf_diff_matrix[2, 3]],
                    [0.0, 0.0, 0.0, 0.0]
                ]
            else:
                theta = math.acos((np.trace(rot_mat) - 1.0) / 2.0)
                eye = np.eye(3, 3)
                m1 = (eye - log3 / 2.0 + (1.0 / theta - 1.0 / math.tan(theta / 2) / 2.0) * log3 ** 2 / theta)
                result = np.dot(m1, trans_vec)
                log = [
                    [log3[0, 0], log3[0, 1], log3[0, 2], result[0]],
                    [log3[1, 0], log3[1, 1], log3[1, 2], result[1]],
                    [log3[2, 0], log3[2, 1], log3[2, 2], result[2]],
                    [0.0, 0.0, 0.0, 0.0]
                ]

            x_err = np.array([log[2][1], log[0][2], log[1][0], log[0][3], log[1][3], log[2][3]])
            print(x_err)

            self.twists.angular.z = self.pid_w.pid(x_err[2])
            if abs(x_err[2]) < 0.1:
                self.twists.linear.x = self.pid_x.pid(x_err[3])
                self.twists.linear.y = self.pid_y.pid(x_err[4])
            else:
                self.twists.linear.x = 0.0
                self.twists.linear.y = 0.0
            self.twist_publisher.publish(self.twists)

            if self.count < self.plan_d.__len__() and abs(x_err[3]) < 0.1 and abs(x_err[4]) < 0.1:
                self.reference_matrix = self.transform_to_matrix(self.pose_to_transform(self.plan_d[self.count].pose))
                self.count += 1
                self.finish = False

        else:
            self.finish = True
            self.last_pos_angl[1] = msg.pose.pose.position.x
            self.last_pos_angl[0] = msg.pose.pose.position.y
            self.twists.angular.z = 0.0
            self.twists.linear.x = 0.0
            self.twists.linear.y = 0.0
            self.twist_publisher.publish(self.twists)
            

    def onClick_points(self, msg: PoseStamped):
        data_send = Path()
        data_send.header.frame_id = 'odom'
        for i in range(1, self.step_ + 1):
            current_data_x = msg.pose.position.x - self.last_position[0]
            current_data_y = msg.pose.position.y - self.last_position[1]
            
            angle = math.atan2(current_data_y - self.last_pos_angl[0], current_data_x - self.last_pos_angl[1])

            tf_quate = tf_transformations.quaternion_from_euler(0, 0, angle)
            rotation = Quaternion(x=tf_quate[0], y=tf_quate[1], z=tf_quate[2], w=tf_quate[3])

            data = PoseStamped()
            data.pose.position.x = (i * current_data_x / self.step_) + self.last_position[0]
            data.pose.position.y = (i * current_data_y / self.step_) + self.last_position[1]
            data.pose.orientation = rotation

            data_send.poses.append(data)

        self.plan__publisher.publish(data_send)

        self.last_position[0] = msg.pose.position.x
        self.last_position[1] = msg.pose.position.y


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
