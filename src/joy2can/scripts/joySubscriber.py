import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

class MotorSetpointSub(Node):
    def __init__(self):
        super().__init__('motorSetpoint')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_sp',
            self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.spL = 0
        self.spR = 0

    def listener_callback(self, msg):
        # Set deadzone and map msg input to object variables
        self.spL = msg.data[0]
        self.spR = msg.data[1]