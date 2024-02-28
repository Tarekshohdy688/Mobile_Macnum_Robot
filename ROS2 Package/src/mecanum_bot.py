#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class MecanumInverseModel(Node):

    def __init__(self):
        super().__init__('mecanum_bot')
        self.motor_pwm_publisher = self.create_publisher(Float32MultiArray, '/wheel_vel', 10)
        self.subscription = self.create_subscription(Twist,'/diff_cont/cmd_vel_unstamped', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.publish_pwm)

        # Define PWM conversion parameters
        self.pwm_min = 100.0  
        self.pwm_max = 255.0
        self.max_vel = 20.0 
        self.max_w = 2.0    

        # Initialize v_x, v_y, and w with default values
        self.v_x = 0.0
        self.v_y = 0.0
        self.w = 0.0


    def listener_callback(self, msg2):
        self.v_x = msg2.linear.x
        self.v_y = msg2.linear.y
        self.w = msg2.angular.z
        self.get_logger().info('Your robot x speed is: "%s"' % self.v_x)
        self.get_logger().info('Your robot y speed is: "%s"' % self.v_y)
        self.get_logger().info('Your robot w speed is: "%s"' % self.w)

        
    def publish_pwm(self):
            
            # Take wheel speeds output from wheel_calc()
            self.wheel_vel = list(self.wheel_calc().flatten())

            # Convert wheel speeds to PWM values
            wheel_pwm = self.convert_speed_to_pwm(self.wheel_vel)
            
            # Publish PWM values
            pwm_msg = Float32MultiArray()
            pwm_msg.data = wheel_pwm
            self.motor_pwm_publisher.publish(pwm_msg)
            self.get_logger().info('Publishing wheel speeds: "%s"' % pwm_msg.data)


    def convert_speed_to_pwm(self, vel_list):
        pwm_values = []
        for vel in vel_list:
            if vel > 0:
                pwm_values.append(max(self.pwm_min, min(self.pwm_max, (vel / self.max_vel) * self.pwm_max)))
            elif vel < 0:
                pwm_values.append(max(self.pwm_min, min(self.pwm_max, (vel*-1 / self.max_vel) * self.pwm_max))*-1)
            else:
                pwm_values.append(0.0)

        return pwm_values


    def wheel_calc(self):
        self.mat1 = np.array([[0.707, -0.707, -0.181],
                            [-0.707, -0.707, 0.181],
                            [0.707, -0.707, 0.181],
                            [-0.707, -0.707, -0.181]])
        self.mat2 = np.linalg.inv(np.array([[0.042, 0, 0, 0],
                                            [0, -0.042, 0, 0],
                                            [0, 0, 0.042, 0],
                                            [0, 0, 0, -0.042]]))
        self.mat3 = np.array([[self.v_x], [self.v_y], [self.w]])
        self.res1 = np.dot(self.mat2, self.mat1)
        self.res2 = np.dot(self.res1, self.mat3)
        return self.res2
    

def main(args=None):
    rclpy.init(args=args)
    mecanum_bot = MecanumInverseModel()
    rclpy.spin(mecanum_bot)
    mecanum_bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()