import numpy as np
import cv2
from scipy import io
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from utils import ros2_utils, tf_utils

STAGE_FIRST_FRAME = 0
STAGE_SECOND_FRAME = 1
STAGE_DEFAULT_FRAME = 2
kMinNumFeature = 1500

FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=50)

sift = cv2.SIFT_create()
flann = cv2.FlannBasedMatcher(index_params, search_params)

def featureTracking(image_ref, image_cur, kp_ref, des_ref):
    """
    Tracks features using SIFT + FLANN-based matcher with Lowe’s Ratio Test.
    """
    
    kp_cur, des_cur = sift.detectAndCompute(image_cur, None)

    if des_ref is None or des_cur is None or len(kp_cur) == 0:
        return np.array([]), np.array([]), None, None

    # FLANN parameters
    matches = flann.knnMatch(des_ref, des_cur, k=2)

    # Lowe’s Ratio Test
    good_matches = []
    for m, n in matches:
        if m.distance < 0.7 * n.distance:
            good_matches.append(m)

    px_ref = np.float32([kp_ref[m.queryIdx].pt for m in good_matches])
    px_cur = np.float32([kp_cur[m.trainIdx].pt for m in good_matches])

    return px_ref, px_cur, kp_cur, des_cur

def estimate_scale(px_ref, px_cur):
    """
    Estimate scale using keypoint displacement ratio.

    Parameters:
    - px_ref: Keypoints from frame t-1 (previous)
    - px_cur: Keypoints from frame t   (current)

    Returns:
    - scale: Estimated scale factor
    """
    # Compute distances of corresponding points
    dist = np.linalg.norm(px_cur - px_ref, axis=1)

    # Avoid division by zero
    dist = dist[dist > 0]

    scale = np.mean(dist)
    return scale

class PUB_CAM_VO(Node):
    def __init__(self, params):
        super().__init__(params["node_name"])
        qos_profile = ros2_utils.custom_qos_profile(params["queue_size"])
        self.frame_stage = STAGE_FIRST_FRAME
        self.new_frame = None
        self.last_frame = None
        self.cur_R = np.eye(3)
        self.cur_t = np.zeros((3, 1))
        self.px_ref = None
        self.px_cur = None
        self.kp_ref = None
        self.des_ref = None
        self.sift = cv2.SIFT_create()
        self.br = CvBridge()
        self.pub_cam_pose = ros2_utils.publisher(
            self, PoseStamped, 'pose', qos_profile
        )
        self.sub_cam_info = ros2_utils.subscriber(
            self, CameraInfo, 'rect/camera_info', self.cam_info_cb, qos_profile
        )
        self.sub_cam_img = ros2_utils.subscriber(
            self, CompressedImage, 'rect/image_color', self.img_cb, qos_profile
        )
		
    def cam_info_cb(self, caminfo_msg):
        self.K = np.array(caminfo_msg.k).reshape(3,3)

    def gen_pose_msg(self, R, T, frame_id='espcam'):
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = ros2_utils.now(self)
        quat = tf_utils.R2quat(R)
        msg.pose.position.x = T[0]
        msg.pose.position.y = T[1]
        msg.pose.position.z = T[2]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        return msg

    def processFirstFrame(self):
        self.kp_ref, self.des_ref = self.sift.detectAndCompute(self.new_frame, None)
        self.px_ref = np.array([kp.pt for kp in self.kp_ref], dtype=np.float32)
        if self.px_ref.size != 0:
            self.frame_stage = STAGE_SECOND_FRAME

    def processSecondFrame(self):
        self.px_ref, self.px_cur, self.kp_ref, self.des_ref = featureTracking(self.last_frame, self.new_frame, self.kp_ref, self.des_ref)
        if len(self.px_ref) < 10:  # Avoid bad initialization
            return
        E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, self.K, method=cv2.RANSAC, prob=0.999, threshold=1)
        _, self.cur_R, self.cur_t, _ = cv2.recoverPose(E, self.px_cur, self.px_ref, self.K)
        self.frame_stage = STAGE_DEFAULT_FRAME
        self.px_ref = self.px_cur

    def processFrame(self):
        self.px_ref, self.px_cur, self.kp_ref, self.des_ref = featureTracking(self.last_frame, self.new_frame, self.kp_ref, self.des_ref)
        
        if len(self.px_ref) < 10:  # Avoid bad tracking updates
            return

        E, _ = cv2.findEssentialMat(self.px_cur, self.px_ref, self.K, method=cv2.RANSAC, prob=0.999, threshold=1)
        _, R, t, _ = cv2.recoverPose(E, self.px_cur, self.px_ref, self.K)
        
        scale = estimate_scale(self.px_ref, self.px_cur)
        ros2_utils.loginfo(self, f'scale: {scale}')
        self.cur_t = self.cur_t + scale * (self.cur_R @ t)  # Apply scale
        self.cur_R = R @ self.cur_R
        
        if len(self.px_ref) < kMinNumFeature:
            self.kp_ref, self.des_ref = self.sift.detectAndCompute(self.new_frame, None)
            self.px_ref = np.array([kp.pt for kp in self.kp_ref], dtype=np.float32)

        pose_msg = self.gen_pose_msg(self.cur_R, self.cur_t.reshape(-1))
        self.pub_cam_pose.publish(pose_msg)

    def img_cb(self, img_msg):
        self.new_frame = self.br.compressed_imgmsg_to_cv2(img_msg)
        if self.frame_stage == STAGE_DEFAULT_FRAME:
            self.processFrame()
        elif self.frame_stage == STAGE_SECOND_FRAME:
            self.processSecondFrame()
        elif self.frame_stage == STAGE_FIRST_FRAME:
            self.processFirstFrame()
        self.last_frame = self.new_frame
