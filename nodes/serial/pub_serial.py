# ROS
from rclpy.node import Node

# Python
import numpy as np
import requests

# Custom
from elegoo_src_ros2_interfaces.msg import Ultrasonic
from utils import ros2_utils

class PUB_SERIAL(Node):
    def __init__(self, params):
        super().__init__(params['node_name'])
        self.url = f"http://{params['esp_ip']}/serial"
        qos_profile = ros2_utils.custom_qos_profile(
            params['queue_size']
        )
        self.pub_ultrasonic = ros2_utils.publisher(
            self, Ultrasonic, "ultrasonic", qos_profile
        )
        self.timer = ros2_utils.timer(
            self, 1/params['max_hz'], self.callback
        )

    def callback(self):
         response = requests.get(self.url)
         if "Distance" in response.text:
            msg = Ultrasonic()
            msg.stamp = ros2_utils.now(self)
            msg.data = float(response.text.split('\t')[1])
            self.pub_ultrasonic.publish(msg)