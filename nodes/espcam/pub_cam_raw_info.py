import numpy as np
from scipy import io

from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

from utils import ros2_utils

class PUB_CAM_RAW_INFO(Node):
    def __init__(self, params):
        super().__init__(params["node_name"])
        qos_profile = ros2_utils.custom_qos_profile(params["queue_size"])
        self.pub_cam_info = ros2_utils.publisher(
            self, CameraInfo, 'raw/camera_info', qos_profile
        )
        self.cam_info_msg = self.load_cam_info(
            params["frame_id"],
            params['calib_type'],
            params['calib_path']
        )
        self.timer = self.create_timer(1/params['fps'], self.callback)

    def load_cam_info(self, frame_id, calib_type, calib_path):
        msg = CameraInfo()
        if 'matlab' in calib_type:
            calib_data = io.loadmat(calib_path, simplify_cells=True)['data']
            msg.header.frame_id  = frame_id
            msg.header.stamp     = ros2_utils.now(self)
            msg.width            = int(calib_data['ImageSize'][1])
            msg.height           = int(calib_data['ImageSize'][0])
            msg.distortion_model = 'plumb_bob'
            msg.d                = list([
                calib_data['RadialDistortion'][0],
                calib_data['RadialDistortion'][1],
                calib_data['TangentialDistortion'][0],
                calib_data['TangentialDistortion'][1],
                calib_data['RadialDistortion'][2],
            ])
            msg.k                = list(calib_data['K'].ravel())
            msg.r                = list(np.eye(3).ravel())
            msg.p                = list([
                calib_data['FocalLength'][0], 0, calib_data['PrincipalPoint'][0], 0,
                0, calib_data['FocalLength'][1], calib_data['PrincipalPoint'][1], 0,
                0, 0, 1, 0
            ])
        return msg

    def callback(self):
        if self.cam_info_msg is None:
            return
        self.cam_info_msg.header.stamp = ros2_utils.now(self)
        self.pub_cam_info.publish(self.cam_info_msg)
