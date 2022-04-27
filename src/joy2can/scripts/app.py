#!/usr/bin/env python
from flask import Flask, render_template, request
import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Int8

template_dir = '../../../../src/joy2can/scripts/templates'
static_dir = '../../../../src/joy2can/scripts/templates/static'

app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)

class WebPublisher(Node):

    def __init__(self):
        super().__init__('web_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'web_data', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.frame_width = 0
        self.frame_height = 0
        self.wall_width = 0
        self.wall_height = 0
        self.x_offset = 0
        self.z_offset = 0
        self.percent_overlap = 0

        # 0 = stop_job
        # 1 = pause_job
        # 2 = start_job
        self.job_status = 0

    def timer_callback(self):
        msg = Float32MultiArray()

        msg.data = [float(self.frame_width),
                    float(self.frame_height),
                    float(self.wall_width),
                    float(self.wall_height),
                    float(self.x_offset),
                    float(self.z_offset),
                    float(self.percent_overlap),
                    float(self.job_status)]
        self.publisher_.publish(msg)






if __name__ == '__main__':
    rclpy.init(args=None)
    wp = WebPublisher()
    rclpy.spin_once(wp)
    @app.route("/")
    def home():
        return render_template("index.html")


    @app.route('/update_parameters', methods=['POST'])
    def update_parameters():
        if request.method == 'POST':
            wp.frame_width = request.form['frame_width']
            wp.frame_height = request.form['frame_height']
            wp.wall_width = request.form['wall_width']
            wp.wall_height = request.form['wall_height']
            wp.x_offset = request.form['x_offset']
            wp.z_offset = request.form['z_offset']
            wp.percent_overlap = request.form['percent_overlap']
            rclpy.spin_once(wp)
            return render_template("index.html", frame_width=wp.frame_width,
                                                 frame_height=wp.frame_height,
                                                 wall_width=wp.wall_width,
                                                 wall_height=wp.wall_height,
                                                 x_offset=wp.x_offset,
                                                 z_offset=wp.z_offset,
                                                 percent_overlap=wp.percent_overlap)

    @app.route('/control')
    def control():
            return render_template("control.html", frame_width=wp.frame_width)

    @app.route('/start_job', methods=['POST', 'GET'])
    def start_job():
        wp.job_status = 2
        rclpy.spin_once(wp)
        return render_template("control.html", job_status=wp.job_status)

    @app.route('/pause_job', methods=['POST', 'GET'])
    def pause_job():
        wp.job_status = 1
        rclpy.spin_once(wp)
        return render_template("control.html", job_status=wp.job_status)

    @app.route('/stop_job', methods=['POST', 'GET'])
    def stop_job():
        wp.job_status = 0
        rclpy.spin_once(wp)
        return render_template("control.html", job_status=wp.job_status)


    app.debug = True
    app.config['EXPLAIN_TEMPLATE_LOADING'] = True
    app.run(debug=True, port=5002, host='0.0.0.0')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wp.destroy_node()
    rclpy.shutdown()

