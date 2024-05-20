#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.qos import qos_profile_system_default


class Commander(Node):

    def __init__(self):
        super().__init__('commander')

        timer_period = 0.02
        self.enable = False
        self.data_scale_x = 0.5
        self.data_scale_y = 0.5
        self.data_scale_z = 1.0

        self.pub_pos = self.create_publisher(Twist, '/cmd_vel', qos_profile=qos_profile_system_default)
        self.subscription = self.create_subscription(Joy, 'joy', self.listener_callback, qos_profile=qos_profile_system_default)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        if self.declare_parameter('enable_button_require', rclpy.Parameter.Type.INTEGER):
            self.button_req = self.get_parameter('enable_button_require').value
        
        if self.declare_parameter('data_scale_x', rclpy.Parameter.Type.DOUBLE):
            self.data_scale_x = self.get_parameter('data_scale_x').value

        if self.declare_parameter('data_scale_y', rclpy.Parameter.Type.DOUBLE):
            self.data_scale_y = self.get_parameter('data_scale_y').value

        if self.declare_parameter('data_scale_z', rclpy.Parameter.Type.DOUBLE):
            self.data_scale_z = self.get_parameter('data_scale_z').value
            
        self.get_logger().info(f"enable_button_require: {self.button_req}")
        self.get_logger().info(f"scale x: {self.data_scale_x}")
        self.get_logger().info(f"scale y: {self.data_scale_y}")
        self.get_logger().info(f"scale z: {self.data_scale_z}")

        self.desired_x = 0.0
        self.desired_y = 0.0
        self.desired_yaw = 0.0

        self.desired_x_prev = 0.0
        self.desired_y_prev = 0.0
        self.desired_yaw_prev = 0.0

    def timer_callback(self):
        msg_data = Twist()
        msg_data.linear.x = self.desired_x
        msg_data.linear.y = self.desired_y
        msg_data.angular.z = self.desired_yaw
        if self.enable == True:
            self.pub_pos.publish(msg_data)
            self.desired_x_prev = msg_data.linear.x
            self.desired_y_prev = msg_data.linear.y
            self.desired_yaw_prev = msg_data.angular.z
        else:
            if self.desired_x_prev != 0 or self.desired_y_prev != 0 or self.desired_yaw_prev != 0:
                self.desired_x_prev = 0.0
                self.desired_y_prev = 0.0
                self.desired_yaw_prev = 0.0
                msg_data.linear.x = 0.0
                msg_data.linear.y = 0.0
                msg_data.angular.z = 0.0
                self.pub_pos.publish(msg_data)

    def listener_callback(self, data):
        self.desired_x = data.axes[1]*self.data_scale_x
        self.desired_y = data.axes[0]*self.data_scale_y
        self.desired_yaw = data.axes[3]*self.data_scale_z
        self.enable = data.buttons[self.button_req]



def main(args=None):
    rclpy.init(args=None)
    commander = Commander()
    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()