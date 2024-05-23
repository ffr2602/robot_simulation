#!/usr/bin/python3

import rclpy
import math
import numpy as np
import rclpy.parameter
import tf_transformations

from geometry_msgs.msg import PoseStamped, Twist, TransformStamped, Pose, Quaternion, Point
from nav_msgs.msg import Odometry, Path
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
from visualization_msgs.msg import Marker

from robot_simulation.pid import Pid


class node_maker(Node):
    plan_d = Path()
    twists = Twist()

    pid_x = Pid(10.0, 0.5, 20.0)
    pid_y = Pid(10.0, 0.5, 20.0)
    pid_w = Pid(5.0, 0, 0.2)

    count = 0
    step_ = 500

    int_x_err = 0.0
    last_pos_angl = np.zeros(2)
    last_position = np.zeros(2)
    finish = True

    def __init__(self):
        super().__init__('path_tracking')
        self.get_logger().info('start node')

        self.declare_parameter('orientation', value='follow_line')

        self.create_subscription(PoseStamped, '/goal_pose', self.onClick_points, qos_profile=qos_profile_system_default)
        # self.create_subscription(Odometry, '/odom', self.onOdom_data, qos_profile=qos_profile_system_default)
        self.create_subscription(Path, '/plan', self.onPlan, qos_profile=qos_profile_system_default)
        self.plan__publisher = self.create_publisher(Path, '/plan', qos_profile=qos_profile_system_default)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile=qos_profile_system_default)
        self.mark__publisher = self.create_publisher(Marker, '/marker', qos_profile=qos_profile_system_default)
        self.odref_publisher = self.create_publisher(Odometry, '/odom_ref', qos_profile=qos_profile_system_default)

        self.marker_setting()

    def onPlan(self, msg: Path):
        self.plan_d.poses += msg.poses
        self.finish = False

    def pose_to_transform(self, msg: Pose):
        transform = TransformStamped()
        transform.transform.translation.x = msg.position.x
        transform.transform.translation.y = msg.position.y
        transform.transform.translation.z = msg.position.z
        transform.transform.rotation = msg.orientation
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
    
    def odom_ref(self, position: Pose):
        odometry_refrence = Odometry()
        odometry_refrence.header.stamp = self.get_clock().now().to_msg()
        odometry_refrence.header.frame_id = "odom"
        odometry_refrence.child_frame_id = "base_footprint"
        odometry_refrence.pose.pose.position.x = position.position.x
        odometry_refrence.pose.pose.position.y = position.position.y
        odometry_refrence.pose.pose.orientation = position.orientation
        self.odref_publisher.publish(odometry_refrence)

    def onOdom_data(self, msg: Odometry):

        if self.finish is not True and self.count < self.plan_d.poses.__len__() - 1:

            global log, log3, x_err

            current___config = self.transform_to_matrix(self.pose_to_transform(msg.pose.pose))      
            reference_config = self.transform_to_matrix(self.pose_to_transform(self.plan_d.poses[self.count].pose)) 
            next______config = self.transform_to_matrix(self.pose_to_transform(self.plan_d.poses[self.count + 1].pose)) 

            current_matrix_inv = np.linalg.inv(current___config)
            tf_diff_matrix = np.dot(current_matrix_inv, reference_config)
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
                    vec = np.array([rot_mat[0, 2], rot_mat[1, 2], 1.0 + rot_mat[2, 2]])
                    omg = (1.0 / math.sqrt(2.0 * (1.0 + rot_mat[2, 2]))) * vec
                elif 1.0 + rot_mat[1, 1] > 0.0001 or -1.0 - rot_mat[1, 1] < -0.0001:
                    vec = [rot_mat[0, 1], 1.0 + rot_mat[1, 1], rot_mat[2, 1]]
                    omg = (1.0 / math.sqrt(2.0 * (1.0 + rot_mat[1, 1]))) * vec
                else:
                    vec = [1.0 + rot_mat[0, 0], rot_mat[1, 0], rot_mat[2, 0]]
                    omg = (1.0 / math.sqrt(2.0 * (1.0 + rot_mat[0, 0]))) * vec
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
            self.get_logger().info(str(x_err[2]) + " " + str(x_err[3]) + " " + str(x_err[4]) + " " + str(self.count) + " " + str(self.plan_d.poses.__len__()))

            tf_diff_matrix = np.dot(np.linalg.inv(reference_config), next______config)
            tf_diff_matrix *= 50.0
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
                if 1.0 + rot_mat[2, 2] > 0.0001 or -1.0 - rot_mat[2, 2] < -0.0001:
                    vec = np.array([rot_mat[0, 2], rot_mat[1, 2], 1.0 + rot_mat[2, 2]])
                    omg = (1.0 / math.sqrt(2.0 * (1.0 + rot_mat[2, 2]))) * vec
                elif 1.0 + rot_mat[1, 1] > 0.0001 or -1.0 - rot_mat[1, 1] < -0.0001:
                    vec = [rot_mat[0, 1], 1.0 + rot_mat[1, 1], rot_mat[2, 1]]
                    omg = (1.0 / math.sqrt(2.0 * (1.0 + rot_mat[1, 1]))) * vec
                else:
                    vec = [1.0 + rot_mat[0, 0], rot_mat[1, 0], rot_mat[2, 0]]
                    omg = (1.0 / math.sqrt(2.0 * (1.0 + rot_mat[0, 0]))) * vec
                m = math.pi * omg
                log3 = [
                    [0.0, -m[2], m[1]],
                    [m[2], 0.0, -m[0]],
                    [-m[1], m[0], 0.0],
                ]
            else:
                theta = math.acos(acosinput)
                log3 = (theta / 2.0 / math.sin(theta)) * (rot_mat - rot_mat.transpose())

            if self.is_zero(log3, 0.0000000000001):
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

            v_d = np.array([log[2][1], log[0][2], log[1][0], log[0][3], log[1][3], log[2][3]])

            # finish logaritma

            tf_diff_matrix = np.dot(np.linalg.inv(current___config), reference_config)
            rot_mat = np.array(tf_diff_matrix[:3, :3])

            so3 = [
                [0.0, -tf_diff_matrix[2,3], tf_diff_matrix[1,3]],
                [tf_diff_matrix[2,3], 0.0, -tf_diff_matrix[0,3]],
                [tf_diff_matrix[1,3], tf_diff_matrix[0,3], 0.0]
            ]

            mat_dot = np.dot(so3, rot_mat)

            adj = [
                [rot_mat[0,0], rot_mat[0,1], rot_mat[0,2], 0.0, 0.0, 0.0],
                [rot_mat[1,0], rot_mat[1,1], rot_mat[1,2], 0.0, 0.0, 0.0],
                [rot_mat[2,0], rot_mat[2,1], rot_mat[2,2], 0.0, 0.0, 0.0],
                [mat_dot[0,0], mat_dot[0,1], mat_dot[0,2], rot_mat[0,0], rot_mat[0,1], rot_mat[0,2]],
                [mat_dot[1,0], mat_dot[1,1], mat_dot[1,2], rot_mat[1,0], rot_mat[1,1], rot_mat[1,2]],
                [mat_dot[2,0], mat_dot[2,1], mat_dot[2,2], rot_mat[2,0], rot_mat[2,1], rot_mat[2,2]]
            ]

            ff_term = np.dot(adj, v_d)
            p_term = x_err * 50.0
            
            if np.linalg.norm(self.int_x_err + x_err * 1.0 / 50.0) < 0.01:
                self.int_x_err = self.int_x_err + x_err * 1.0 / 50.0
            
            i_term = self.int_x_err * 1.0
            p_control = p_term + i_term + ff_term

            self.twists.angular.z = p_control[2]
            self.twists.linear.y = p_control[4]
            self.twists.linear.x = p_control[3]
          
            self.twist_publisher.publish(self.twists)
            self.count += 1
    
            # if self.plan_count > 0:
            #     if self.count == (self.plan_count * self.step_) - 1:
            #         self.twists.angular.z = self.pid_w.pid(x_err[2])
            #         if abs(x_err[2]) < 0.015:
            #             self.count += 1

            #     elif self.count == self.plan_d.poses.__len__() - 1:
            #         self.twists.linear.x = self.pid_x.pid(x_err[3])
            #         self.twists.linear.y = self.pid_y.pid(x_err[4])
            #         if abs(x_err[3]) < 0.015 and abs(x_err[4]) < 0.015:
            #             self.twists.angular.z = self.pid_w.pid(x_err[2])
            #             if abs(x_err[2]) < 0.015:
            #                 self.finish = True
            #                 self.get_logger().info('selesai')
            #                 self.plan_count += 0.5
                    
            #     else:
            #         self.twists.angular.z = self.pid_w.pid(x_err[2])
            #         self.twists.linear.x = self.pid_x.pid(x_err[3])
            #         self.twists.linear.y = self.pid_y.pid(x_err[4])
            #         self.count += 1

            # else:
            #     if self.count == self.plan_count * self.step_:
            #         self.twists.angular.z = self.pid_w.pid(x_err[2])
            #         if abs(x_err[2]) < 0.015:
            #             self.count += 1

            #     elif self.count == self.plan_d.poses.__len__() - 1:
            #         self.twists.linear.x = self.pid_x.pid(x_err[3])
            #         self.twists.linear.y = self.pid_y.pid(x_err[4])
            #         if abs(x_err[3]) < 0.015 and abs(x_err[4]) < 0.015:
            #             self.twists.angular.z = self.pid_w.pid(x_err[2])
            #             if abs(x_err[2]) < 0.0015:
            #                 self.finish = True
            #                 self.get_logger().info('selesai')
            #                 self.plan_count += 0.5
                    
            #     else:
            #         self.twists.angular.z = self.pid_w.pid(x_err[2])
            #         self.twists.linear.x = self.pid_x.pid(x_err[3])
            #         self.twists.linear.y = self.pid_y.pid(x_err[4])
            #         self.count += 1

            # # self.twists.linear.x = self.pid_x.pid(x_err[3])
            # # self.twists.linear.y = self.pid_y.pid(x_err[4])
            # self.twist_publisher.publish(self.twists)

            # if abs(x_err[3]) < 0.015 and abs(x_err[4]) < 0.015:
            #     self.count += 1
                # self.odom_ref(self.plan_d[self.count].pose)
                # self.finish = False

        else:
            self.finish = True
            self.last_pos_angl[1] = msg.pose.pose.position.x
            self.last_pos_angl[0] = msg.pose.pose.position.y
            self.twists.angular.z = 0.0
            self.twists.linear.x = 0.0
            self.twists.linear.y = 0.0
            self.twist_publisher.publish(self.twists)

        
            

    def onClick_points(self, msg: PoseStamped):
        self.finish = False
        line_point = Point()
        line_point.x = msg.pose.position.x
        line_point.y = msg.pose.position.y
        self.marker.points.append(line_point)
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.mark__publisher.publish(self.marker)

        data_send = Path()
        data_send.header.frame_id = 'odom'
        for i in range(1, self.step_ + 1):
            data = PoseStamped()
            if self.get_parameter('orientation').value == 'fixed':
                data.pose.position.x = (i * (msg.pose.position.x - self.last_position[0]) / self.step_) + self.last_position[0]
                data.pose.position.y = (i * (msg.pose.position.y - self.last_position[1]) / self.step_) + self.last_position[1]

            elif self.get_parameter('orientation').value == 'follow_line':
                data.pose.position.x = (i * (msg.pose.position.x - self.last_position[0]) / self.step_) + self.last_position[0]
                data.pose.position.y = (i * (msg.pose.position.y - self.last_position[1]) / self.step_) + self.last_position[1]
                angle = math.atan2((msg.pose.position.y - self.last_position[1]) - self.last_pos_angl[0], (msg.pose.position.x - self.last_position[0]) - self.last_pos_angl[1])
                tf_quate = tf_transformations.quaternion_from_euler(0, 0, angle)
                data.pose.orientation = Quaternion(x=tf_quate[0], y=tf_quate[1], z=tf_quate[2], w=tf_quate[3])

            elif self.get_parameter('orientation').value == 'setpoint':
                data.pose.position.x = (i * (msg.pose.position.x - self.last_position[0]) / self.step_) + self.last_position[0]
                data.pose.position.y = (i * (msg.pose.position.y - self.last_position[1]) / self.step_) + self.last_position[1]
                data.pose.orientation = msg.pose.orientation

            elif self.get_parameter('orientation').value == 'follow_line_&_setpoint':
                if i < self.step_ -1:
                    data.pose.position.x = (i * (msg.pose.position.x - self.last_position[0]) / self.step_) + self.last_position[0]
                    data.pose.position.y = (i * (msg.pose.position.y - self.last_position[1]) / self.step_) + self.last_position[1]
                    angle = math.atan2((msg.pose.position.y - self.last_position[1]) - self.last_pos_angl[0], (msg.pose.position.x - self.last_position[0]) - self.last_pos_angl[1])
                    tf_quate = tf_transformations.quaternion_from_euler(0, 0, angle)
                    data.pose.orientation = Quaternion(x=tf_quate[0], y=tf_quate[1], z=tf_quate[2], w=tf_quate[3])
                else:
                    data.pose.position.x = (i * (msg.pose.position.x - self.last_position[0]) / self.step_) + self.last_position[0]
                    data.pose.position.y = (i * (msg.pose.position.y - self.last_position[1]) / self.step_) + self.last_position[1]
                    data.pose.orientation = msg.pose.orientation
            
            data_send.poses.append(data)

        self.plan__publisher.publish(data_send)

        self.last_position[0] = msg.pose.position.x
        self.last_position[1] = msg.pose.position.y
    
    def marker_setting(self):
        self.marker = Marker()
        self.marker.header.frame_id = "odom"
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD

        self.marker.scale.x = 0.03
        self.marker.scale.y = 0.03
        self.marker.scale.z = 0.01

        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.points = []
        line_point = Point()
        line_point.x = 0.0
        line_point.y = 0.0
        self.marker.points.append(line_point)


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