import math
import rclpy
import numpy as np
from matplotlib import pyplot as plt
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualizator_msgs.msg import Marker

class PSM(Node):
        def __init__(self):
            super().__init__('psm_grasp')

            self.measured_cp = None
            self.measured_jaw = None
            self.marker = None

            # Subscribers
            self.subscription_cp = self.create_subscription(
                PoseStamped,
                '/PSM1/measured_cp',
                self.cb_meassured_cp,
                10)

            self.subscription_jaw = self.create_subscription(
                JointState,
                '/PSM1/jaw/measured_js',
                self.cb_meassured_jaw,
                10)

            self.subscription_marker = self.create_subscription(
                Marker,
                '/dummy_target_marker',
                self.cb_marker,
                10)

            # Publishers
            self.pub_cp = self.create_publisher(
                PoseStamped,
                '/PSM1/servo_cp',
                10)

            self.pub_jaw = self.create_publisher(
                JointState,
                '/PSM1/jaw/servo_jp',
                10)

        # Callback for pose
        def cb_meassured_cp(self, msg):
            self.measured_cp = msg
            #print(self.measured_cp)

        # Callback for jaw
        def cb_meassured_jaw(self, msg):
            self.measured_jaw = msg
            #print(self.measured_jaw)

        # Callback for marker
        def cb_meassured_jaw(self, msg):
            self.marker = msg
            #print(self.measured_jaw)

        def move_tcp_to(self, target, v, dt):
            # Wait for position to be received
            loop_rate = self.create_rate(100, self.get_clock()) # Hz
            while self.measured_cp is None and rclpy.ok():
                self.get_logger().info('Waiting for pose...')
                rclpy.spin_once(self)

            pose_msg = self.measured_cp

            # Calculate trajectory parameters
            pos_curr_np = np.array([self.measured_cp.pose.position.x,
                                   self.measured_cp.pose.position.y,
                                   self.measured_cp.pose.position.z])

            pos_target_np = np.array(target)

            distance = np.linalg.norm(pos_target_np - pos_curr_np)

            T = distance / v
            N = int(math.floor(T / dt))

            arr_x = np.linespace(pos_curr_np[0], pos_target_np[0], N)
            arr_y = np.linespace(pos_curr_np[1], pos_target_np[1], N)
            arr_z = np.linespace(pos_curr_np[2], pos_target_np[2], N)

            # Play the trajectory
            for i in range(N):
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.pose.position.x = arr_x[i]
                pose_msg.pose.position.y = arr_y[i]
                pose_msg.pose.position.z = arr_z[i]

                self.pub_cp.publish(pose_msg)
                print("Motion command sent.")

            print("TCP arrived to desired position.")


        # Open/close grippers
        def move_jaw_to(self, target, omega, dt):
            # Wait for position to be received
            loop_rate = self.create_rate(1.0 / dt, self.get_clock()) # Hz
            while self.measured_jaw is None and rclpy.ok():
                self.get_logger().info('Waiting for jaw...')
                rclpy.spin_once(self)


            # Create the msg to be sent
            jaw_msg = self.measured_jaw


            # Calculate trajectory parameters
            distance = abs(self.measured_jaw.position[0] - target)
            T = distance / omega
            N = int(math.floor(T / dt))

            arr_jaw = np.linspace(self.measured_jaw.position[0], target, N)

            # Play the trajectory
            for i in range(N):
                jaw_msg.header.stamp = self.get_clock().now().to_msg
                jaw_msg.position = [arr_jaw[i]]

                self.jaw_cp.publish(jaw_msg)
                rclpy.spin_once(self)

            print("Jaws arrived to desired position.")


        def grasp(self, v, omega, dt):
            # Wait for marker to be received
            loop_rate = self.create_rate(1.0 / dt, self.get_clock()) # Hz
            while self.marker is None and rclpy.ok():
                self.get_logger().info('Waiting for marker...')
                rclpy.spin_once(self)

            # Get marker position
            target = [self.marker.pose.position.x,
                      self.marker.pose.position.y,
                      self.marker.pose.position.z + 0.008]

            # Open jaws
            self.move_jaw_to(0.8, omega, dt)

            # Move TCP to marker
            self.move_tcp_to(target, v, dt)

            # Close the jaws
            self.move_jaw_to(0.2. omega, dt)


def main(args=None):
    rclpy.init(args=args)
    psm = PSM()

    # Reset the arm
    psm.move_tcp_to([0.0, 0.0, -0.12], 0.01, 0.01)
    psm.move_jaw_to(0.0, 0.1, 0.01)

    # Perform the motion
    #psm.move_tcp_to([0.0, 0.05, -0.12], 0.01, 0.01)
    #psm.move_jaw_to(0.8, 0.1, 0.01)
    psm.grasp(0.01, 0.1, 0.01)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    psm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
