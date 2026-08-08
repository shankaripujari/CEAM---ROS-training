import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class DiffDrivePIDController(Node):
    def __init__(self):
        super().__init__('diff_drive_pid_controller')
        
        # PID Parameters
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('ki', 0.05)
        self.declare_parameter('kd', 0.1)
        
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        
        # State tracking variables
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.current_linear_velocity = 0.0
        
        self.integral_error = 0.0
        self.previous_error = 0.0

        # Subscriptions & Publishers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.left_wheel_pub = self.create_publisher(Float64, '/left_wheel_cmd', 10)
        self.right_wheel_pub = self.create_publisher(Float64, '/right_wheel_cmd', 10)
        
        self.timer = self.create_timer(0.05, self.control_loop) # 20 Hz loop
        self.get_logger().info('Differential Drive PID Controller Node Started.')

    def cmd_callback(self, msg: Twist):
        self.target_linear_velocity = msg.linear.x
        self.target_angular_velocity = msg.angular.z

    def odom_callback(self, msg: Odometry):
        self.current_linear_velocity = msg.twist.twist.linear.x

    def control_loop(self):
        error = self.target_linear_velocity - self.current_linear_velocity
        self.integral_error += error * 0.05
        derivative_error = (error - self.previous_error) / 0.05
        
        # Compute PID output
        pid_output = (self.kp * error) + (self.ki * self.integral_error) + (self.kd * derivative_error)
        self.previous_error = error

        # Inverse Kinematics (Wheel separation = 0.5m)
        wheel_separation = 0.5
        left_cmd = pid_output - (self.target_angular_velocity * wheel_separation / 2.0)
        right_cmd = pid_output + (self.target_angular_velocity * wheel_separation / 2.0)

        msg_left = Float64()
        msg_right = Float64()
        msg_left.data = left_cmd
        msg_right.data = right_cmd

        self.left_wheel_pub.publish(msg_left)
        self.right_wheel_pub.publish(msg_right)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDrivePIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
