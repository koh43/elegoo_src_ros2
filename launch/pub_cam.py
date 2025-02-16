import launch
import launch_ros.actions

def generate_launch_description():
    params = {
        "node_name": "pub_espcam_raw",
        "ns" : "/espcam",
        "queue_size": 5,
        "cam_ip": "192.168.1.103",
        "cam_port": 81,
        "frame_id": "espcam",
        "fps": 30
    }

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="elegoo_src_ros2",
            executable="pub_cam_raw",
            name=params["node_name"],
            namespace=params['ns'],
            output="screen",
            parameters=[params]  # Pass the dictionary as parameters
        )
    ])