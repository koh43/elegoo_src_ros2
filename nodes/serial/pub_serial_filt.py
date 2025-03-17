# ROS
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Python
import time
import numpy as np
from filterpy.kalman import KalmanFilter
from ahrs.filters import Madgwick

# Custom
from elegoo_src_ros2_interfaces.msg import Ultrasonic, Voltage, LineTracker
from utils import ros2_utils

class PUB_SERIAL_FILT(Node):
    def __init__(self, params):
        super().__init__(params['node_name'])
        qos_profile = ros2_utils.custom_qos_profile(
            params['queue_size']
        )
        self.sub_imu_raw = ros2_utils.subsciber(
            self, Imu, "raw/imu", self.imu_cb, qos_profile
        )
        self.pub_imu_filt = ros2_utils.publisher(
            self, Imu, "filt/imu", qos_profile
        )
        # self.pub_ultrasonic = ros2_utils.publisher(
        #     self, Ultrasonic, "ultrasonic", qos_profile
        # )
        # self.pub_voltage = ros2_utils.publisher(
        #     self, Voltage, "voltage", qos_profile
        # )
        # self.pub_line_tracker = ros2_utils.publisher(
        #     self, LineTracker, "linetracker", qos_profile
        # )
        self.load_constants()
        self.init_flag = True
        self.madgwick = Madgwick()
        self.prev_quat = np.array([1.0, 0.0, 0.0, 0.0])

    def load_constants(self):
        self.accel_sensitivity = 16384.0 # 2g
        self.gyro_sensitivity = 131.0 # ±250°/s
        self.gravity = 9.80665
        self.deg2rad = np.pi / 180.0

    def load_kfs(self, imu_msg, process_variance=1e-5):
        init_acc = [
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ]
        init_gyro = [
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        ]
        self.acc_kf = KalmanFilter(dim_x=3, dim_z=3)
        self.acc_kf.F = np.eye(3)  # State transition matrix (no dynamics)
        self.acc_kf.H = np.eye(3)  # Measurement matrix
        self.acc_kf.R = np.array(imu_msg.linear_acceleration_covariance).reshape(3,3)
        self.acc_kf.Q = np.eye(3) * process_variance  # Process noise
        self.acc_kf.x = np.array(init_acc)

        self.gyro_kf = KalmanFilter(dim_x=3, dim_z=3)
        self.gyro_kf.F = np.eye(3)  # State transition matrix (no dynamics)
        self.gyro_kf.H = np.eye(3)  # Measurement matrix
        self.gyro_kf.R = np.array(imu_msg.angular_velocity_covariance).reshape(3,3)
        self.gyro_kf.Q = np.eye(3) * process_variance  # Process noise
        self.gyro_kf.x = np.array(init_gyro)

    def imu_cb(self, imu_msg):
        if self.init_flag:
            self.load_kfs(imu_msg)
            self.init_flag = False
            return
        self.acc_kf.predict()
        self.acc_kf.update([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ])
        filt_acc = self.acc_kf.x[:3]

        self.gyro_kf.predict()
        self.gyro_kf.update([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        ])
        filt_gyro = self.gyro_kf.x[:3]

        self.prev_quat = self.madgwick.updateIMU(self.prev_quat, gyr=filt_gyro, acc=filt_acc)

        msg = Imu()
        msg.header.frame_id = "my_src"
        msg.header.stamp = ros2_utils.now(self)
        msg.orientation.w = self.prev_quat[0]
        msg.orientation.x = self.prev_quat[1]
        msg.orientation.y = self.prev_quat[2]
        msg.orientation.z = self.prev_quat[3]
        msg.linear_acceleration.x = filt_acc[0]
        msg.linear_acceleration.y = filt_acc[1]
        msg.linear_acceleration.z = filt_acc[2]
        msg.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance
        msg.angular_velocity.x = filt_gyro[0]
        msg.angular_velocity.y = filt_gyro[1]
        msg.angular_velocity.z = filt_gyro[2]
        msg.angular_velocity_covariance = imu_msg.angular_velocity_covariance
        self.pub_imu_filt.publish(msg)
