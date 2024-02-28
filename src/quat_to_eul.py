#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math


class QuatToEulNode(Node):
    def __init__(self):
        super().__init__('quat_to_eul')
        self.subscription = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.publisher_euler = self.create_publisher(Twist, '/euler_angles', 10)

    def imu_callback(self, msg):
        # Get quaternion from IMU message
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        # Convert quaternion to Euler angles
        euler_angles = self.quat_to_eul(quaternion)

        # Create Twist message with Euler angles
        euler_msg = Twist()
        euler_msg.angular.x = euler_angles[0]
        euler_msg.angular.y = euler_angles[1]
        euler_msg.angular.z = euler_angles[2]

        # Publish the Euler angles
        self.publisher_euler.publish(euler_msg)

    def quat_to_eul(self, quaternion):
        # Convert quaternion to Euler angles using ZYX rotation sequence
        psi = math.atan2(2 * (quaternion[1] * quaternion[2] + quaternion[3] * quaternion[0]),
                         (quaternion[0] * quaternion[0] - quaternion[1] * quaternion[1] -
                          quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]))
        theta_arg = 2 * (quaternion[1] * quaternion[3] - quaternion[2] * quaternion[0])
        theta_arg = max(min(theta_arg, 1.0), -1.0)  # Clip the argument to the valid range [-1, 1]
        theta = math.asin(theta_arg)
        phi = math.atan2(2 * (quaternion[2] * quaternion[3] + quaternion[1] * quaternion[0]),
                         (quaternion[0] * quaternion[0] - quaternion[1] * quaternion[1] -
                          quaternion[2] * quaternion[2] - quaternion[3] * quaternion[3]))

        return [psi, theta, phi]


def main(args=None):
    rclpy.init(args=args)
    quat_to_eul = QuatToEulNode()
    rclpy.spin(quat_to_eul)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
