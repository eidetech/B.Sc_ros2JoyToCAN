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
		self.px = 0
		self.pz = 0
		self.reset = 0.0;

	def listener_callback(self, msg):
		self.spL = msg.data[1]
		self.spR = msg.data[0]
		#self.px = msg.data[2]
		#self.pz = msg.data[3]
