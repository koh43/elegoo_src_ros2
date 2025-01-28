from setuptools import find_packages, setup

package_name = 'ros2_src'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koh',
    maintainer_email='sktlgt93@gmail.com',
    description='ROS2 Package for ELEGOO Smart Robot Car',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)
