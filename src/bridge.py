#!/usr/bin/env python3
import zmq
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class Bridge(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, '/angles', 10)
        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/wheel_vel',
            self.listener_callback,
            10)
        
        c1 = zmq.Context()
        self.sub = c1.socket(zmq.PAIR)

        c2 = zmq.Context()
        self.pub = c2.socket(zmq.PAIR)
        self.pub.bind('tcp://127.0.0.1:1234')


    def timer_callback(self):
        self.sub.connect('tcp://127.0.0.1:1122')
        data = self.sub.recv().decode("utf-8")
        print(data)
        imu = data.split(",")
        wz = float(imu[0])
        yaw = float(imu[1])
        imu_msg = Float32MultiArray()
        imu_msg.data = [wz, yaw]
        self.publisher.publish(imu_msg)

    def listener_callback(self, msg):
        phi = msg.data
        phi1 = phi[0]
        phi2 = phi[1]
        phi3 = phi[2]
        phi4 = phi[3]
        self.pub.send(f"{phi1},{phi2},{phi3},{phi4}".encode("utf-8"))

def main(args=None):
    rclpy.init(args=args)

    minimal = Bridge()
    rclpy.spin(minimal)

    minimal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()