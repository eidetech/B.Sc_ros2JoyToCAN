import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoySubscriber(Node):

    def __init__(self):
        super().__init__('joySubscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.joyX = 0
        self.joyZ = 0

    def listener_callback(self, msg):
        # Set deadzone and map msg input to object variables
        if(msg.axes[0] > 0.1):
            self.joyX = msg.axes[0]
        elif(msg.axes[0] < -0.1):
            self.joyX = msg.axes[0]
        else:
            self.joyX = 0
        if(msg.axes[1] > 0.1):
            self.joyZ = msg.axes[1]
        elif(msg.axes[1] < -0.1):
            self.joyZ = msg.axes[1]
        else:
            self.joyZ = 0