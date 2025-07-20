from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'elegoo_src_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('launch/*.xml')),
    ],
    install_requires=[
        'setuptools',
        'elegoo_src_ros2_interfaces',
    ],
    zip_safe=True,
    maintainer='koh',
    maintainer_email='sktlgt93@gmail.com',
    description='ROS2 Package for ELEGOO Smart Robot Car',
    license='MIT',
    entry_points={
        'console_scripts': [
            # espcam
            'pub_cam_raw      = elegoo_src_ros2.espcam.pub_cam_raw:main',
            'pub_cam_raw_info = elegoo_src_ros2.espcam.pub_cam_raw_info:main',
            'pub_cam_rect     = elegoo_src_ros2.espcam.pub_cam_rect:main',
            'pub_cam_vo       = elegoo_src_ros2.espcam.pub_cam_vo:main',
            'cap_calib_imgs   = elegoo_src_ros2.espcam.cap_calib_imgs:main',
            # arduino serial
            'pub_serial_raw  = elegoo_src_ros2.serial.pub_serial_raw:main',
            'pub_serial_filt = elegoo_src_ros2.serial.pub_serial_filt:main',
            # misc
            'image_view      = elegoo_src_ros2.misc.image_view:main',
        ],
    },
)
