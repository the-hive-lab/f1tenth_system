#Import ROS libraries
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from std_msgs.msg import Float64

#Import some utils.
import numpy as np
import copy as copy

class InterpolateThrottle(Node):
    def __init__(self):
        super().__init__('throttle_interpolator')
        
        #Declaring parameters
        self.declare_parameter('rpm_input_topic')
        self.declare_parameter('rpm_output_topic')
        self.declare_parameter('servo_input_topic')
        self.declare_parameter('servo_output_topic')
        self.declare_parameter('max_acceleration')
        self.declare_parameter('throttle_smoother_rate')
        self.declare_parameter('speed_max')
        self.declare_parameter('speed_min')
        self.declare_parameter('speed_to_erpm_gain')
        self.declare_parameter('max_servo_speed')
        self.declare_parameter('steering_angle_to_servo_gain')
        self.declare_parameter('servo_smoother_rate')
        self.declare_parameter('servo_max')
        self.declare_parameter('servo_min')
        self.declare_parameter('steering_angle_to_servo_offset')

        #Reading in parameters for node, by default the parameters can be found
        #under config/params.yaml
        self.rpm_input_topic = self.get_parameter('rpm_input_topic')
        self.rpm_output_topic = self.get_parameter('rpm_output_topic')
        self.servo_input_topic = self.get_parameter('servo_input_topic')
        self.servo_output_topic = self.get_parameter('servo_output_topic')
        self.max_acceleration = self.get_parameter('max_acceleration')
        self.throttle_smoother_rate = self.get_parameter('throttle_smoother_rate')
        self.max_rpm = self.get_parameter('speed_max')
        self.min_rpm = self.get_parameter('speed_min')
        self.speed_to_erpm_gain = self.get_parameter('speed_to_erpm_gain')
        self.max_servo_speed = self.get_parameter('max_servo_speed')
        self.steering_angle_to_servo_gain = self.get_parameter('steering_angle_to_servo_gain')
        self.servo_smoother_rate = self.get_parameter('servo_smoother_rate')
        self.max_servo = self.get_parameter('servo_max')
        self.min_servo = self.get_parameter('servo_min')
        self.last_servo_param = self.get_parameter('steering_angle_to_servo_offset')

        # Variables
        self.last_rpm = 0
        self.desired_rpm = self.last_rpm
        self.desired_servo_position = self.last_servo_param.value
        self.last_servo = self.last_servo_param.value

        # Create topic publishers for throttle and steering commands        
        self.rpm_output = self.create_publisher(Float64, self.rpm_output_topic.value, 1)
        self.servo_output = self.create_publisher(Float64, self.servo_output_topic.value, 1)

        # Create topic subscribers for throttle and steering commands  
        self.rpm_input = self.create_subscription(Float64, self.rpm_input_topic.value, self._process_throttle_command, 1)
        self.rpm_input  # prevent unused variable warning
        self.servo_input = self.create_subscription(Float64, self.servo_input_topic.value, self._process_servo_command, 1)
        self.servo_input  # prevent unused variable warning        

        #Create timers for throttle.
        self.max_delta_rpm = abs(self.speed_to_erpm_gain.value * self.max_acceleration.value / self.throttle_smoother_rate.value)
        self.timer = self.create_timer((1.0/self.max_delta_rpm), self._publish_throttle_command)

        #Create timers for steering.
        self.max_delta_servo = abs(self.steering_angle_to_servo_gain.value * self.max_servo_speed.value / self.servo_smoother_rate.value)
        self.timer = self.create_timer((1.0/self.servo_smoother_rate.value), self._publish_servo_command)
      
    def _publish_throttle_command(self):
        desired_delta = self.desired_rpm-self.last_rpm
        clipped_delta = max(min(desired_delta, self.max_delta_rpm), -self.max_delta_rpm)
        smoothed_rpm = Float64()
        smoothed_rpm.data = float(self.last_rpm + clipped_delta)
        self.last_rpm = smoothed_rpm.data      
        self.rpm_output.publish(smoothed_rpm)
            
    def _process_throttle_command(self,msg):
        input_rpm = msg.data
        # Do some sanity clipping
        input_rpm = min(max(input_rpm, self.min_rpm.value), self.max_rpm.value)
        self.desired_rpm = input_rpm

    def _publish_servo_command(self):
        desired_delta = self.desired_servo_position - self.last_servo
        clipped_delta = max(min(desired_delta, self.max_delta_servo), -self.max_delta_servo)
        smoothed_servo = Float64()
        smoothed_servo.data = self.last_servo + clipped_delta
        self.last_servo = smoothed_servo.data         
        self.servo_output.publish(smoothed_servo)

    def _process_servo_command(self,msg):
        input_servo = msg.data
        # Do some sanity clipping
        input_servo = min(max(input_servo, self.min_servo.value), self.max_servo.value)
        # set the target servo position
        self.desired_servo_position = input_servo

def main(args=None):
    rclpy.init(args=args)
    throttle_interpolator = InterpolateThrottle()
    rclpy.spin(throttle_interpolator)
    throttle_interpolator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()