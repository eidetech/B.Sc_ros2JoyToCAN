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
		self.angVel_q1 = 0
		self.angVel_q2 = 0
		self.angPos_q1 = 0
		self.angPos_q2 = 0

		self.angPos_q1_initial = 0
		self.angPos_q2_initial = 0

		self.mode = 1 # Start system in position mode
		self.lastMode = 0
		self.t = 0
		self.total_t = 0

	def listener_callback(self, msg):
		self.angVel_q1 = -msg.data[0] # Negative due to motor mounting.
		self.angVel_q2 = -msg.data[1] # Negative due to motor mounting.
		self.angPos_q1 = -msg.data[2] # Negative due to motor mounting.
		self.angPos_q2 = -msg.data[3] # Negative due to motor mounting.
		self.mode = 	  msg.data[4]
		self.t = 		  msg.data[5]
		self.t_total =    msg.data[6]
		if (msg.data[7] == 1.0):
			self.angPos_q1_initial = -msg.data[2] # Negative due to motor mounting.
			self.angPos_q2_initial = -msg.data[3] # Negative due to motor mounting.
