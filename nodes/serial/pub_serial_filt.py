# ROS
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

# Python
import time
import numpy as np
import sys
print(sys.path)
from filterpy.kalman import KalmanFilter
from ahrs.filters import Madgwick
from ahrs import Quaternion

# Custom
from utils import ros2_utils

class PUB_SERIAL_FILT(Node):
    def __init__(self, params):
        super().__init__(params['node_name'])
        qos_profile = ros2_utils.custom_qos_profile(
            params['queue_size']
        )
        self.sub_imu_raw = ros2_utils.subscriber(
            self, Imu, "raw/imu", self.imu_cb, qos_profile
        )
        self.pub_imu_filt = ros2_utils.publisher(
            self, Imu, "filt/imu", qos_profile
        )
        self.load_constants()
        self.init_flag = True
        self.madgwick = Madgwick()
        self.prev_quat = Quaternion([1.0, 0.0, 0.0, 0.0])

    def load_constants(self):
        self.accel_sensitivity = 16384.0 # 2g
        self.gyro_sensitivity = 131.0 # ±250°/s
        self.gravity = 9.80665
        self.deg2rad = np.pi / 180.0

    def load_kfs(self, imu_msg, process_variance=1e-6, delta_t=0.01):
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
        # self.acc_kf = KalmanFilter(dim_x=9, dim_z=3)
        # self.acc_kf.F = np.eye(9)   # State transition matrix
        # self.acc_kf.F[:3, 3:6] = self.acc_kf.F[3:6, 6:] = np.eye(3)*delta_t
        # self.acc_kf.F[:3, 6:] = np.eye(3)*0.5*delta_t*delta_t
        # self.acc_kf.H = np.zeros((3, 9)) # Measurement matrix
        # self.acc_kf.H[:, 6:] = np.eye(3)
        # self.acc_kf.R = np.array(imu_msg.linear_acceleration_covariance).reshape(3, 3)
        # self.acc_kf.Q = np.eye(9) * process_variance  # Process noise
        # self.acc_kf.x = np.hstack([np.zeros(6), init_acc])

        self.acc_kf = KalmanFilter(dim_x=3, dim_z=3)
        self.acc_kf.F = np.eye(3)   # State transition matrix (no dynamics)
        self.acc_kf.H = np.eye(3)
        self.acc_kf.R = np.array(imu_msg.linear_acceleration_covariance).reshape(3, 3)
        self.acc_kf.Q = np.eye(3) * process_variance  # Process noise
        self.acc_kf.x = np.array(init_acc)

        self.gyro_kf = KalmanFilter(dim_x=3, dim_z=3)
        self.gyro_kf.F = np.eye(3)  # State transition matrix (no dynamics)
        self.gyro_kf.H = np.eye(3)  # Measurement matrix
        self.gyro_kf.R = np.array(imu_msg.angular_velocity_covariance).reshape(3,3)
        self.gyro_kf.Q = np.eye(3) * process_variance  # Process noise
        self.gyro_kf.x = np.array(init_gyro)
    
    # def remove_gravity(self, acc_imu, prev_quat):        
    #     # Gravity vector in world frame (assuming Z-up)
    #     g_world = Quaternion([0, 0, 0, self.gravity])  # (0, 0, 9.81)
        
    #     # Rotate gravity into IMU frame: g_imu = q* ⊗ g_world ⊗ q
    #     g_imu = prev_quat*g_world
    #     g_imu = g_imu*prev_quat.conj

    #     # Subtract gravity from IMU acceleration
    #     return acc_imu - g_imu[1:]
    
    def publish_imu(self, quat, accel, gyro, linear_cov, ang_cov):
        msg = Imu()
        msg.header.frame_id = "my_src"
        msg.header.stamp = ros2_utils.now(self)
        msg.orientation.w = quat[0]
        msg.orientation.x = quat[1]
        msg.orientation.y = quat[2]
        msg.orientation.z = quat[3]
        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]
        msg.linear_acceleration_covariance = linear_cov
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]
        msg.angular_velocity_covariance = ang_cov
        self.pub_imu_filt.publish(msg)

    # def publish_odometry(self, lin_pos, lin_vel, gyro, quat):
    #     msg = Odometry()
    #     msg.header.stamp = ros2_utils.now(self)
    #     msg.header.frame_id = "map"  # World frame
    #     msg.child_frame_id = "base_link"  # Robot's frame

    #     # Set Position
    #     msg.pose.pose.position.x = lin_pos[0]
    #     msg.pose.pose.position.y = lin_pos[1]
    #     msg.pose.pose.position.z = lin_pos[2]

    #     # Set Orientation (quaternion)
    #     msg.pose.pose.orientation.w = quat[0]
    #     msg.pose.pose.orientation.x = quat[1]
    #     msg.pose.pose.orientation.y = quat[2]
    #     msg.pose.pose.orientation.z = quat[3]

    #     # Set Velocity
    #     msg.twist.twist.linear.x = lin_vel[0]
    #     msg.twist.twist.linear.y = lin_vel[1]
    #     msg.twist.twist.linear.z = lin_vel[2]

    #     # Set Angular Velocity (from IMU)
    #     msg.twist.twist.angular.x = gyro[0]
    #     msg.twist.twist.angular.y = gyro[1]
    #     msg.twist.twist.angular.z = gyro[2]

    #     self.pub_odom.publish(msg)

    def imu_cb(self, imu_msg):
        if self.init_flag:
            self.load_kfs(imu_msg)
            self.init_flag = False
            return
        self.acc_kf.predict()
        z_acc = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ])
        # z_acc_grav_comp = self.remove_gravity(z_acc, self.prev_quat)
        z_acc_grav_comp = z_acc
        self.acc_kf.update(z_acc_grav_comp)
        # filt_pos = self.acc_kf.x[:3]
        # filt_vel = self.acc_kf.x[3:6]
        # filt_acc = self.acc_kf.x[6:]
        filt_acc = self.acc_kf.x[:3]
        
        self.gyro_kf.predict()
        self.gyro_kf.update([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        ])
        filt_gyro = self.gyro_kf.x[:3]

        self.prev_quat = self.madgwick.updateIMU(self.prev_quat, gyr=filt_gyro, acc=filt_acc)

        self.publish_imu(
            self.prev_quat,
            filt_acc,
            filt_gyro,
            self.acc_kf.P.flatten(),
            self.gyro_kf.P.flatten()
        )

        # self.publish_odometry(
        #     filt_pos,
        #     filt_vel,
        #     filt_gyro,
        #     self.prev_quat
        # )
