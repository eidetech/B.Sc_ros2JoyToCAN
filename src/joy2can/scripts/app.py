#!/usr/bin/env python
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import json
import os
import git
from git import repo
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

# Define templates path and static files path
template_dir = '../../../../src/joy2can/scripts/templates'
static_dir = '../../../../src/joy2can/scripts/templates/static'

# Configure Flask application with paths
app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)
app.config['JSON_SORT_KEYS'] = False
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)


# ROS2 node for publishing data from Flask server website
class WebPublisher(Node):

	def __init__(self):
		super().__init__('web_publisher')
		self.publisher_ = self.create_publisher(Float32MultiArray, 'web_data', 10)
		timer_period = 0.1  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)

		# Parameters for the painting job
		self.frame_width = 0
		self.frame_height = 0
		self.wall_width = 0
		self.wall_height = 0
		self.x_offset = 0
		self.z_offset = 0
		self.vertical_step = 0

		self.counter = 0

		# 0 = stop_job
		# 1 = pause_job
		# 2 = start_job
		self.job_status = 0

	def timer_callback(self):
		msg = Float32MultiArray()

		# Fill msg array with data
		msg.data = [float(self.frame_width),
					float(self.frame_height),
					float(self.wall_width),
					float(self.wall_height),
					float(self.x_offset),
					float(self.z_offset),
					float(self.vertical_step),
					float(self.job_status)]
		wp.counter += 1
		# Publish data to ROS
		self.publisher_.publish(msg)


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

		self.firstIteration = True

		self.reset = 0

	def listener_callback(self, msg):
		self.angVel_q1 = msg.data[0]
		self.angVel_q2 = msg.data[1]
		self.angPos_q1 = msg.data[2]
		self.angPos_q2 = msg.data[3]
		self.mode = 	 msg.data[4]
		self.t = 		 msg.data[5]

if __name__ == '__main__':
	# Initialize ROS node and web publisher object
	rclpy.init()
	wp = WebPublisher()
	motorSP = MotorSetpointSub()



	# App route for the homepage (index.html)
	@app.route("/")
	def home():
		return render_template("index.html", job_status=0,
							   frame_width=wp.frame_width,
							   frame_height=wp.frame_height,
							   wall_width=wp.wall_width,
							   wall_height=wp.wall_height,
							   x_offset=wp.x_offset,
							   z_offset=wp.z_offset,
							   vertical_step=wp.vertical_step)

	# App route for POST request updating the local parameters with data from the website
	@app.route('/update_parameters', methods=['POST'])
	def update_parameters():
		if request.method == 'POST':
			wp.frame_width = request.form['frame_width']
			wp.frame_height = request.form['frame_height']
			wp.wall_width = request.form['wall_width']
			wp.wall_height = request.form['wall_height']
			wp.x_offset = request.form['x_offset']
			wp.z_offset = request.form['z_offset']
			wp.vertical_step = request.form['vertical_step']
			wp.job_status = 0
			rclpy.spin_once(wp)
			return render_template("index.html", job_status= 0,
								   frame_width=wp.frame_width,
								   frame_height=wp.frame_height,
								   wall_width=wp.wall_width,
								   wall_height=wp.wall_height,
								   x_offset=wp.x_offset,
								   z_offset=wp.z_offset,
								   vertical_step=wp.vertical_step)

	# App route for the control page (control.html)
	@app.route('/control')
	def control():
		return render_template("control.html", frame_width=float(wp.frame_width))

	# App route for starting the job (POST request)
	@app.route('/start_job', methods=['POST'])
	def start_job():
		wp.job_status = 2
		rclpy.spin_once(wp)
		return render_template("control.html", job_status=wp.job_status, frame_width=float(wp.frame_width))

	# App route for pausing the job (POST request)
	@app.route('/pause_job', methods=['POST'])
	def pause_job():
		wp.job_status = 1
		rclpy.spin_once(wp)
		return render_template("control.html", job_status=wp.job_status,  frame_width=float(wp.frame_width))

	# App route for stopping the job (POST request)
	@app.route('/stop_job', methods=['POST'])
	def stop_job():
		wp.job_status = 0
		rclpy.spin_once(wp)
		return render_template("control.html", job_status=wp.job_status, frame_width=float(wp.frame_width))

	# App route for loading the default parameters from json file default.json
	@app.route('/load_default_params', methods=['POST', 'GET'])
	def load_default_params():
		default_params_path = os.path.join(os.path.dirname(__file__), "default.json")
		with open(default_params_path) as f:
			data = json.load(f)

		wp.frame_width = data['default']['frame_width']
		wp.frame_height = data['default']['frame_height']
		wp.wall_width = data['default']['wall_width']
		wp.wall_height = data['default']['wall_height']
		wp.x_offset = data['default']['x_offset']
		wp.z_offset = data['default']['z_offset']
		wp.vertical_step = data['default']['vertical_step']
		wp.job_status = data['default']['job_status']
		print("wp frame:", data['default']['frame_width'])
		return render_template("index.html", job_status=float(wp.job_status),
							   frame_width=float(wp.frame_width),
							   frame_height=float(wp.frame_height),
							   wall_width=float(wp.wall_width),
							   wall_height=float(wp.wall_height),
							   x_offset=float(wp.x_offset),
							   z_offset=float(wp.z_offset),
							   vertical_step=float(wp.vertical_step))

	# App route for pulling the latest commit from GitHub
	@app.route('/git_pull',  methods=['POST'])
	def git_pull():
		g = git.cmd.Git('/home/marcus/github/B.Sc_ros2JoyToCAN')
		g.pull()
		repo = git.Repo(search_parent_directories=True)
		hash = repo.head.object.hexsha
		return render_template("index.html", git_pull_done=True, hash=hash[:7])

	@socketio.on('angVel')
	def test():
		emit('angVel_q1', motorSP.angVel_q1)
		emit('angVel_q2', motorSP.angVel_q2)
		percent = motorSP.t/66 * 100
		emit('t', percent)

	# Prints out where Flask is looking for the template folder. Useful for debugging if Flask can not find index.html for example
	# app.config['EXPLAIN_TEMPLATE_LOADING'] = True
	executor = rclpy.executors.MultiThreadedExecutor()
	executor.add_node(wp)
	executor.add_node(motorSP)


	# threading.Thread(target=lambda: executor.spin()).start()
	executor_thread = threading.Thread(target=executor.spin, daemon=True)
	executor_thread.start()

	socketio.run(app, debug=True, use_reloader=False,  port=5002, host='0.0.0.0')

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	rclpy.shutdown()
	executor_thread.join()

