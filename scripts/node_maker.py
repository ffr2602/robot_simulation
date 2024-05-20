import rclpy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node

from robot_simulation.pid import Pid


class node_maker(Node):
    finish = False
    twists = Twist()
    pid_x = Pid(7.0, 0, 0.2)
    pid_y = Pid(7.0, 0, 0.2)
    pid_w = Pid(7.0, 0, 0.25)
    position = 0
    step = 100
    last_position = np.zeros(3)
    
    data_plan = [PoseStamped()]
    data_raw = [PoseStamped()]

    def __init__(self):
        super().__init__('node_maker')
        self.get_logger().info('start node')
    
        self.pid_x.sp = self.data_plan[self.position].pose.position.x
        self.pid_y.sp = self.data_plan[self.position].pose.position.y
        self.pid_w.sp = self.data_plan[self.position].pose.orientation.w

        self.create_subscription(PoseStamped, '/goal_pose', self.onClick_points, qos_profile=qos_profile_system_default)
        self.create_subscription(Odometry, '/odom', self.onOdom_data, qos_profile=qos_profile_system_default)
        self.create_subscription(Path, '/plan', self.onPlan, qos_profile=qos_profile_system_default)
        self.plan__publisher = self.create_publisher(Path, '/plan', qos_profile=qos_profile_system_default)
        self.maker_publisher = self.create_publisher(Marker, '/maker', qos_profile=qos_profile_system_default)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile=qos_profile_system_default)
        self.maker_setting()

        self.data_point = 0.0

    def onPlan(self, msg: Path):
        self.data_plan += msg.poses
        print(self.data_plan)

    def onOdom_data(self, msg: Odometry):
        self.pid_x.pos = msg.pose.pose.position.x
        self.pid_y.pos = msg.pose.pose.position.y
        self.pid_w.pos = msg.pose.pose.orientation.w
        if self.finish is not True:
            self.twists.linear.x = self.pid_x.pid()
            self.twists.linear.y = self.pid_y.pid()
            self.twists.angular.z = self.pid_w.pid()
            self.twist_publisher.publish(self.twists)

        if abs(self.pid_x.sp - self.pid_x.pos) < 0.015 and abs(self.pid_y.sp - self.pid_y.pos) < 0.015 and abs(self.pid_w.sp - self.pid_w.pos) < 0.015:
            if self.position < self.data_plan.__len__():
                self.pid_x.reset_err()
                self.pid_y.reset_err()
                self.pid_w.reset_err()
                self.pid_x.sp = self.data_plan[self.position].pose.position.x
                self.pid_y.sp = self.data_plan[self.position].pose.position.y
                self.pid_w.sp = self.data_plan[self.position].pose.orientation.w
                self.position += 1
                self.finish = False
            else:
                self.finish = True
                self.twists.linear.x = 0.0
                self.twists.linear.y = 0.0
                self.twists.angular.z = 0.0
                self.twist_publisher.publish(self.twists)

    def onClick_points(self, msg: PoseStamped):
        data_send = Path()
        for i in range(1, self.step + 1):
            current_data_1 = msg.pose.position.x - self.last_position[0]
            current_data_2 = msg.pose.position.y - self.last_position[1]
            data = PoseStamped()
            data.pose.position.x = (i * current_data_1 / self.step) + self.last_position[0]
            data.pose.position.y = (i * current_data_2 / self.step) + self.last_position[1]
            # data.pose.orientation.w = i * msg.pose.orientation.w / self.step
            data_send.poses.append(data)

        self.plan__publisher.publish(data_send)

        self.last_position[0] = msg.pose.position.x
        self.last_position[1] = msg.pose.position.y

        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.maker_publisher.publish(self.marker)

    def maker_setting(self):
        self.marker = Marker()
        self.marker.header.frame_id = "odom"
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD

        # self.marker scale
        self.marker.scale.x = 0.03
        self.marker.scale.y = 0.03
        self.marker.scale.z = 0.03

        # self.marker color
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

        # self.marker orientaiton
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.points = []
        # line_point = Point()
        # line_point.x = 0.0
        # line_point.y = 0.0
        # self.marker.points.append(line_point)


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
