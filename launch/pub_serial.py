import launch
import launch_ros.actions

def generate_launch_description():
    params = {
        "node_name": "pub_arduino_serial",
        "ns" : "/arduino/serial",
        "queue_size": 5,
        "esp_ip": "192.168.1.103",
        "max_hz": 1000
    }

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="elegoo_src_ros2",
            executable="pub_arduino_serial",
            name=params["node_name"],
            namespace=params['ns'],
            output="screen",
            parameters=[params]  # Pass the dictionary as parameters
        )
    ])