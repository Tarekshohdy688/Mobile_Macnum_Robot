#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class MecanumDrive(Node):

    def __init__(self):
        super().__init__('mecanum_bot')
        self.motor_pwm_publisher = self.create_publisher(Float32MultiArray, '/wheel_vel', 10)
        self.subscription_imu = self.create_subscription(Float32MultiArray, '/angles', self.cmd_imu_callback, 10)
        self.subscription_vel = self.create_subscription(Twist,'/diff_cont/cmd_vel_unstamped', self.cmd_vel_callback, 10)
        self.subscription_params = self.create_subscription(Float32MultiArray,'/pid_cont_params', self.cmd_param_callback, 10)
        self.subscription_vel
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_pwm)

        self.new_velocity_command_received = False

        # Define PWM conversion parameters
        self.pwm_min = 0.0  
        self.pwm_max = 200.0
        self.max_vel = 20.0 
        self.max_w = 2.0  

        # Initialize velocities with default values
        self.v_x = 0.0
        self.v_y = 0.0

        # State Variables
        self.current_angle = 0.0
        self.current_w = 0.0
        self.desired_w = 0.0 
        self.previous_angle = 0.0
        self.e_int = 0.0
        self.e_prev = 0.0

        # PID control parameters
        self.kp = 0.1
        self.ki = 0.0
        self.kd = 0.0
        self.percent = 0.0

        self.desired_yaw = 0.0
        self.flag = 0

    def cmd_param_callback(self, k_msg):
            self.kp = k_msg.data[0]
            self.ki = k_msg.data[1]
            self.kd = k_msg.data[2]
            self.percent = k_msg.data[3]

            # self.get_logger().info('Parameters: kp=%f, ki=%f, kd=%f, percent=%f' % (self.kp, self.ki, self.kd, self.percent))

    def cmd_imu_callback(self, imu_msg):
        # Calculate the angular speed (current_w) from euler angle
        self.current_w = imu_msg.data[0]
        self.current_angle = imu_msg.data[1]
    
    def cmd_vel_callback(self, vel_msg):
        self.v_x = vel_msg.linear.x
        self.v_y = vel_msg.linear.y
        self.desired_w = vel_msg.angular.z
        
        if self.desired_w != 0.0:
            self.flag = 0
        else:
            if self.flag == 0:
                self.desired_yaw = self.current_angle
                self.flag +=1

        self.new_velocity_command_received = True

        # self.get_logger().info('Your robot desired x is: "%s"' % self.v_x)
        # self.get_logger().info('Your robot desired y is: "%s"' % self.v_y)
        # self.get_logger().info('Your robot desired w is: "%s"' % self.desired_w)

    def pid_compute(self,wz, desired):
        e = desired - wz
        self.threshold_value = desired * self.percent

        if abs(e) > self.threshold_value:
            self.e_int += e * self.timer_period
            e_dot = (e - self.e_prev) / self.timer_period

            # PID control signal
            pid_output = (
                self.kp * e +
                self.ki * self.e_int +
                self.kd * e_dot
            )
            self.e_prev = e
            
            self.get_logger().info(
                f'Desired: {desired:.2f} rad/s, '
                f'Current: {wz:.2f} rad/s, '
                f'Error: {e:.2f}, '
                f'PID Output: {pid_output:.2f}'
            )
            
            wz = pid_output
            wz = np.clip(wz, -self.max_w, self.max_w)


        return wz
        
    def publish_pwm(self):
        if not self.new_velocity_command_received:
            self.wheel_pwm = [0.0, 0.0, 0.0, 0.0]
            pwm_msg = Float32MultiArray()
            pwm_msg.data = self.wheel_pwm
            self.motor_pwm_publisher.publish(pwm_msg)
            # self.get_logger().info('Publishing wheel velocities: "%s"' % pwm_msg.data)
            return

        # Control Loop
        if self.desired_w != 0.0:
            self.current_w = self.pid_compute(self.current_w, self.desired_w)
        else:
            self.current_w = self.pid_compute(self.current_angle, self.desired_yaw)

        # Robot Inverse Model
        self.wheel_vel = list(self.wheel_calc().flatten())

        # Convert wheel speeds to PWM values
        self.wheel_pwm = self.convert_vel_to_pwm(self.wheel_vel)
        
        # Publish PWM values
        pwm_msg = Float32MultiArray()
        pwm_msg.data = self.wheel_pwm
        self.motor_pwm_publisher.publish(pwm_msg)
        self.new_velocity_command_received = False
        # self.get_logger().info('Publishing wheel velocities: "%s"' % pwm_msg.data)


    def convert_vel_to_pwm(self, vel_list):
        pwm_values = []
        for vel in vel_list:
            if vel >= 0:
                pwm_values.append(max(self.pwm_min, min(self.pwm_max, (vel / self.max_vel) * self.pwm_max)))
            elif vel < 0:
                pwm_values.append(max(self.pwm_min, min(self.pwm_max, (vel*-1 / self.max_vel) * self.pwm_max))*-1)
            else:
                0.0
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
        self.mat3 = np.array([[self.v_x], [self.v_y], [self.current_w]])
        self.res1 = np.dot(self.mat2, self.mat1)
        self.res2 = np.dot(self.res1, self.mat3)

        return self.res2
    

def main(args=None):
    rclpy.init(args=args)
    mecanum_bot = MecanumDrive()
    rclpy.spin(mecanum_bot)
    mecanum_bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()