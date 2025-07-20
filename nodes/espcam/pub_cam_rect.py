import numpy as np
import cv2

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo

from cv_bridge import CvBridge
from utils import ros2_utils

class PUB_CAM_RECT(Node):
    def __init__(self, params):
        super().__init__(params["node_name"])
        qos_profile = ros2_utils.custom_qos_profile(params["queue_size"])
        self.br = CvBridge()
        self.mtx = None
        self.dist = None
        self.pub_rect_img_color = ros2_utils.publisher(
            self, CompressedImage, 'rect/image_color', qos_profile
        )
        self.pub_rect_cam_info = ros2_utils.publisher(
            self, CameraInfo, 'rect/camera_info', qos_profile
        )
        self.sub_raw_cam_info = ros2_utils.subscriber(
            self, CameraInfo, 'raw/camera_info', self.cam_info_cb, qos_profile
        )
        self.sub_raw_img_color = ros2_utils.subscriber(
            self, CompressedImage, 'raw/image_color', self.raw_img_cb, qos_profile
        )

    def cam_info_cb(self, cam_info_msg):
        rect_cam_info_msg = CameraInfo()
        self.mtx = np.array(cam_info_msg.k).reshape(3,3)
        self.dist = np.array(cam_info_msg.d)
        rect_cam_info_msg.width  = cam_info_msg.width
        rect_cam_info_msg.height = cam_info_msg.height
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(
            self.mtx, self.dist,
            (cam_info_msg.width, cam_info_msg.height), 1,
            (cam_info_msg.width, cam_info_msg.height)
        )
        rect_cam_info_msg.d = list(np.zeros(5))
        rect_cam_info_msg.k = list(self.newcameramtx.ravel())
        rect_cam_info_msg.header.frame_id = cam_info_msg.header.frame_id
        rect_cam_info_msg.header.stamp = ros2_utils.now(self)
        rect_cam_info_msg.r = list(np.eye(3).ravel())
        rect_cam_info_msg.p = list([
            self.newcameramtx[0,0], 0, self.newcameramtx[0,2], 0,
            0, self.newcameramtx[1,1], self.newcameramtx[1,2], 0,
            0, 0, 1, 0
        ])
        self.pub_rect_cam_info.publish(rect_cam_info_msg)

    def undistort_img(self, img):
        if self.mtx is None or self.dist is None:
            return None
        dst = cv2.undistort(img, self.mtx, self.dist, None, self.newcameramtx)
        x, y, w, h = self.roi
        dst = dst[y:y+h, x:x+w]
        return dst

    def raw_img_cb(self, raw_img_msg):
        rect_img = self.undistort_img(
            self.br.compressed_imgmsg_to_cv2(raw_img_msg)
        )
        if rect_img is None:
            return
        rect_img_msg = self.br.cv2_to_compressed_imgmsg(rect_img)
        rect_img_msg.header.frame_id = raw_img_msg.header.frame_id
        rect_img_msg.header.stamp = ros2_utils.now(self)
        self.pub_rect_img_color.publish(rect_img_msg)
        