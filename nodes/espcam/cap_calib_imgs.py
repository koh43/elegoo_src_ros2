import cv2
import os

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
from utils import ros2_utils

class CAP_CALIB_IMGS(Node):
    def __init__(self, params):
        super().__init__(params["node_name"])
        qos_profile = ros2_utils.custom_qos_profile(params["queue_size"])
        self.br = CvBridge()
        self.sub_img = ros2_utils.subsciber(
            self, CompressedImage, params['cam_topic_name'], self.callback, qos_profile
        )
        self.chess_size = (5,8)
        self.criteria =(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.save_path = params['save_path']
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
        self.img_count = 0

    def callback(self, img_msg):
        img = self.br.compressed_imgmsg_to_cv2(img_msg)
        ret, corners = cv2.findChessboardCorners(
            img, self.chess_size, 
            cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_NORMALIZE_IMAGE+cv2.CALIB_CB_FAST_CHECK
        )
        if ret:
            ros2_utils.loginfo(self, "Chessboard detected!")
            if self.img_count % 5:
                img_path = os.path.join(
                    self.save_path, f'{self.img_count:5d}.jpg'
                )
                cv2.imwrite(img_path, img)
            self.img_count += 1
