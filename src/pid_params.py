#!/usr/bin/env python3
import rclpy
import rclpy.node 
from std_msgs.msg import Float32MultiArray
from rcl_interfaces.msg import ParameterDescriptor


class Params(rclpy.node.Node):

    def __init__(self):
        super().__init__('pid_params')
        self.params_publisher = self.create_publisher(Float32MultiArray, '/pid_cont_params', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_params)

        param_descriptor_P = ParameterDescriptor(description = "Proportional gain")
        self.declare_parameter ("kp", 1.0, param_descriptor_P)

        param_descriptor_I = ParameterDescriptor(description="Integral gain")
        self.declare_parameter("ki", 0.1, param_descriptor_I)

        param_descriptor_D = ParameterDescriptor(description="Derivative gain")
        self.declare_parameter("kd", 0.01, param_descriptor_D)

        param_descriptor_Percent = ParameterDescriptor(description="Threshold percentage")
        self.declare_parameter("percent", 0.05, param_descriptor_Percent)

    
    def publish_params(self):
        self.kp_param = self.get_parameter("kp").value
        self.ki_param = self.get_parameter("ki").value
        self.kd_param = self.get_parameter("kd").value
        self.percent_param = self.get_parameter("percent").value


        try:
            self.kp_param = float(self.kp_param)
            self.ki_param = float(self.ki_param)
            self.kd_param = float(self.kd_param)
            self.percent_param = float(self.percent_param)
        except ValueError:
            self.get_logger().error("Invalid parameter value. Unable to convert to float.")
            return

        msg = Float32MultiArray()
        msg.data = [self.kp_param, self.ki_param, self.kd_param, self.percent_param]

        self.params_publisher.publish(msg)
        self.get_logger().info('Parameters: kp=%f, ki=%f, kd=%f, percent=%f' % (self.kp_param, self.ki_param, self.kd_param, self.percent_param))


def main(args=None):
    rclpy.init(args=args)
    pid_params = Params()
    rclpy.spin(pid_params)
    pid_params.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()