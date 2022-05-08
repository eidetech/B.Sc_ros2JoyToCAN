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
		self.angPos_q1_final = 0
		self.angPos_q2_final = 0

		self.mode = 0
		self.lastMode = 1
		self.t = 0
		self.total_t = 0

		self.firstIteration = True

		self.reset = 0

	def listener_callback(self, msg):
		self.angVel_q1 = msg.data[0]
		self.angVel_q2 = msg.data[1]
		self.angPos_q1 = msg.data[2]
		self.angPos_q2 = msg.data[3]
		self.mode = 	 msg.data[4]
		self.t = 		 msg.data[5]
		self.t_total =   msg.data[6]

		if(self.firstIteration == True):
			self.angPos_q1_initial = msg.data[2]
			self.angPos_q2_initial = msg.data[3]
			self.firstIteration = False
