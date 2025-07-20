from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
from utils import esp_cam_utils, ros2_utils

class PUB_CAM_RAW(Node):
    def __init__(self, params):
        super().__init__(params["node_name"])
        self.frame_id = params["frame_id"]
        qos_profile = ros2_utils.custom_qos_profile(params["queue_size"])
        self.br = CvBridge()
        self.pub_img_color = self.create_publisher(
            CompressedImage, 'raw/image_color', qos_profile
        )
        self.camera = esp_cam_utils.init_camera(params)
        self.timer = self.create_timer(1/params['fps'], self.callback)

    def callback(self):
        ret, color = self.camera.read()
        if ret:
            color_msg = self.br.cv2_to_compressed_imgmsg(color)
            color_msg.header.frame_id = self.frame_id
            color_msg.header.stamp = ros2_utils.now(self)
            self.pub_img_color.publish(color_msg)