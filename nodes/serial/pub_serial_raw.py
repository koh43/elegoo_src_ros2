# ROS
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Python
import time
import numpy as np
import socket

# Custom
from elegoo_src_ros2_interfaces.msg import Ultrasonic, Voltage, LineTracker
from utils import ros2_utils

class PUB_SERIAL_RAW(Node):
    def __init__(self, params):
        super().__init__(params['node_name'])
        qos_profile = ros2_utils.custom_qos_profile(
            params['queue_size']
        )
        self.pub_imu = ros2_utils.publisher(
            self, Imu, "raw/imu", qos_profile
        )
        self.pub_ultrasonic = ros2_utils.publisher(
            self, Ultrasonic, "raw/ultrasonic", qos_profile
        )
        self.pub_voltage = ros2_utils.publisher(
            self, Voltage, "raw/voltage", qos_profile
        )
        self.pub_line_tracker = ros2_utils.publisher(
            self, LineTracker, "raw/linetracker", qos_profile
        )
        self.timer = ros2_utils.timer(
            self, 1/params['max_hz'], self.callback
        )
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", params['udp_port']))
        self.buffer_size = params['buffer_size']
        self.load_constants()
        self.calc_cov(params['num_calib_samples'])

    def load_constants(self):
        self.accel_sensitivity = 16384.0 # 2g
        self.gyro_sensitivity = 131.0 # ±250°/s
        self.gravity = 9.80665
        self.deg2rad = np.pi / 180.0

    def read_socket(self):
        data, _ = self.sock.recvfrom(self.buffer_size)
        try:
            decoded_data = data.decode()
            return decoded_data
        except:
            return None

    def calc_cov(self, num_calib_samples):
        ros2_utils.loginfo(self, "===== Calculating IMU Covariance =====")
        ros2_utils.loginfo(self, "Please leave the car stationary in a flat surface...")
        acc_x, acc_y, acc_z = [], [], []
        gyro_x, gyro_y, gyro_z = [], [], []
        i = 0
        while(i < num_calib_samples):
            decoded_data = self.read_socket()
            if decoded_data is None:
                continue
            sensor_data = self.get_data(decoded_data)
            if 'IMU' in sensor_data.keys():
                i += 1
                if (i % 500 == 0):
                    ros2_utils.loginfo(self, f"Progress... [{i}/{num_calib_samples}]")
                acc_x.append((float(sensor_data['IMU'][0])/self.accel_sensitivity)*self.gravity)
                acc_y.append((float(sensor_data['IMU'][1])/self.accel_sensitivity)*self.gravity)
                acc_z.append((float(sensor_data['IMU'][2])/self.accel_sensitivity)*self.gravity)
                gyro_x.append(float(sensor_data['IMU'][3])/self.gyro_sensitivity*self.deg2rad)
                gyro_y.append(float(sensor_data['IMU'][4])/self.gyro_sensitivity*self.deg2rad)
                gyro_z.append(float(sensor_data['IMU'][5])/self.gyro_sensitivity*self.deg2rad)
        acc_data = np.vstack([acc_x, acc_y, acc_z])
        gyro_data = np.vstack([gyro_x, gyro_y, gyro_z])
        self.acc_offset = np.mean(acc_data, axis=1)
        self.gyro_offset = np.mean(gyro_data, axis=1)
        acc_var = np.var(acc_data, axis=1)
        gyro_var = np.var(acc_data, axis=1)
        self.acc_cov = [
            acc_var[0], 0, 0,
            0, acc_var[1], 0,
            0, 0, acc_var[2]
        ]
        self.gyro_cov = [
            gyro_var[0], 0, 0,
            0, gyro_var[1], 0,
            0, 0, gyro_var[2]
        ]
        ros2_utils.loginfo(self, "IMU Offset Calibration complete!")

    def get_data(self, data):
        sensor_data = {}
        for sensor_raw_str in data.split('%')[:-1]:
            if len(sensor_raw_str.split(':')) == 2:
                sensor_name, sensor_data_str = sensor_raw_str.split(':')
                for sensor_data_temp in sensor_data_str.split(','):
                    sensor_data[sensor_name] = [float(x) for x in sensor_data_str.split(',')]
                    # if sensor_name in sensor_data.keys():
                    #     sensor_data[sensor_name].append(sensor_data_temp)
                    # else:
                    #     sensor_data[sensor_name] = [sensor_data_temp]
        return sensor_data

    def callback(self):
        # start = time.time()
        decoded_data = self.read_socket()
        if decoded_data is None:
            return
        sensor_data = self.get_data(decoded_data)
        if 'IMU' in sensor_data.keys():
            msg = Imu()
            msg.header.frame_id = "my_src"
            msg.header.stamp = ros2_utils.now(self)
            msg.linear_acceleration.x = (float(sensor_data['IMU'][0])/self.accel_sensitivity)*self.gravity
            msg.linear_acceleration.y = (float(sensor_data['IMU'][1])/self.accel_sensitivity)*self.gravity
            msg.linear_acceleration.z = (float(sensor_data['IMU'][2])/self.accel_sensitivity)*self.gravity - self.gravity
            msg.linear_acceleration_covariance = self.acc_cov
            msg.angular_velocity.x = float(sensor_data['IMU'][3])/self.gyro_sensitivity*self.deg2rad
            msg.angular_velocity.y = float(sensor_data['IMU'][4])/self.gyro_sensitivity*self.deg2rad
            msg.angular_velocity.z = float(sensor_data['IMU'][5])/self.gyro_sensitivity*self.deg2rad
            msg.angular_velocity_covariance = self.gyro_cov
            self.pub_imu.publish(msg)
        if 'US' in sensor_data.keys():
            msg = Ultrasonic()
            msg.stamp = ros2_utils.now(self)
            msg.data = float(sensor_data['US'][0])
            self.pub_ultrasonic.publish(msg)
        if 'V' in sensor_data.keys():
            msg = Voltage()
            msg.stamp = ros2_utils.now(self)
            msg.data = float(sensor_data['V'][0])
            self.pub_voltage.publish(msg)
        if 'LT' in sensor_data.keys():
            msg = LineTracker()
            msg.stamp = ros2_utils.now(self)
            msg.l = float(sensor_data['LT'][0])
            msg.m = float(sensor_data['LT'][1])
            msg.r = float(sensor_data['LT'][2])
            self.pub_line_tracker.publish(msg)
        
        # ros2_utils.loginfo(self, f"elapsed_time: {time.time() - start}")