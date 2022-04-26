#!/usr/bin/env python
from flask import Flask, render_template, request
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Int8

app = Flask(__name__)
app._static_folder = os.path.abspath("/home/marcus/github/B.Sc_ros2JoyToCAN/install/joy2can/lib/joy2can/templates/static/")


class WebPublisher(Node):

    def __init__(self):
        super().__init__('web_publisher')
        self.publisher_ = self.create_publisher(Int8MultiArray, 'web_data', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.frame_width = 0
        self.frame_height = 0
        self.wall_width = 0
        self.wall_height = 0
        self.x_offset = 0
        self.z_offset = 0
        self.percent_overlap = 0

    def timer_callback(self):
        msg = Int8MultiArray()
        msg.data = [int(self.frame_width),
                    int(self.frame_height),
                    int(self.wall_width),
                    int(self.wall_height),
                    int(self.x_offset),
                    int(self.z_offset),
                    int(self.percent_overlap)]
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

    app.debug = True
    app.config['EXPLAIN_TEMPLATE_LOADING'] = True
    app.run(debug=True, port=5002, host='0.0.0.0')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wp.destroy_node()
    rclpy.shutdown()

