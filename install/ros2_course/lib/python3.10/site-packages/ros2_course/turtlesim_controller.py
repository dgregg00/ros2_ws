import math
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')
        self.twist_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose = None
        self.subscription = self.create_subscription(
                Pose,
                '/turtle1/pose',
                self.cb_pose,
                10)

    def cb_pose(self, msg):
        self.pose = msg

    def go_to(self, speed, omega, x, y):
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.pose is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)

        # Stuff with atan2
        x0 = self.pose.x
        y0 = self.pose.y
        theta_0 = math.degrees(self.pose.theta)

        theta_1 = math.degrees(math.atan2(y-y0, x-x0))
        angle = theta_1 - theta_0
        distance = math.sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0))

        self.turn(omega, angle)
        self.go_straight(speed, distance)


    def turn(self, omega, angle):
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.pose is None and rclpy.ok():
            self.get_logger().info("Waiting for pose...")
            rclpy.spin_once(self)


        # Constants
        theta_0 = self.pose.theta
        d_rem = angle
        stop_treshold = 0.1
        p = 0.1

        # Implement straght motion here
        # Create and publish msg
        vel_msg = Twist()

        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = p * d_rem

        # Publish first msg and note time when to stop
        self.twist_pub.publish(vel_msg)
        print("Turtle started.")

        # Publish msg while the calculated time is up
        while (abs(d_rem) > stop_treshold) and rclpy.ok():
            pos_curr = self.pose.theta
            d_rem = angle - math.degrees(pos_curr - theta_0)

            vel_msg.angular.z = p * d_rem
            self.twist_pub.publish(vel_msg)
            print("Remaining: " + str(d_rem))
            rclpy.spin_once(self) # loop rate

        # turtle arrived, set velocity to 0
        vel_msg.angular.z = 0.0


    def go_straight(self, speed, distance):

        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.pose is None and rclpy.ok():
            self.get_logger().info("Waiting for pose...")
            rclpy.spin_once(self)

        # Constant
        pos_0 = np.array([self.pose.x, self.pose.y])
        d_rem = distance
        stop_treshold = 0.01
        p = 5.0

        # Implement straght motion here
        # Create and publish msg
        vel_msg = Twist()
        vel_msg.linear.x = p * d_rem
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        # Publish first msg and note time when to stop
        self.twist_pub.publish(vel_msg)
        print("Turtle started")

        # Publish msg while the calculated time is up
        while (d_rem > stop_treshold) and rclpy.ok():
            pos_curr = np.array([self.pose.x, self.pose.y])
            d_rem = distance - np.linalg.norm(pos_curr - pos_0)

            vel_msg.linear.x = p * d_rem
            self.twist_pub.publish(vel_msg)
            print("Remaining: " + str(d_rem))
            rclpy.spin_once(self) # loop rate


        # turtle arrived, set velocity to 0
        vel_msg.linear.x = 0.0


        def draw_poly(self, speed, omega, N, a):

            angle = 360.0 / N
            for i in range(N):
                self.go_straight(speed, a)
                self.turn(omega, angle)


def main(args=None):
    rclpy.init(args=args)
    tc = TurtlesimController()
    #tc.go_straight(1.0, 4.0)
    #tc.draw_poly(1.0, 100.0, 6, 2.0)
    tc.go_to(1.0, 20.0, 2, 8)
    tc.go_to(1.0, 20.0, 2, 2)
    tc.go_to(1.0, 20.0, 3, 4)
    tc.go_to(1.0, 20.0, 6, 2)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
