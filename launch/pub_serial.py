import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="elegoo_src_ros2",
            executable="pub_serial_raw",
            namespace="/arduino/serial",
            output="screen",
        ),
        launch_ros.actions.Node(
            package="elegoo_src_ros2",
            executable="pub_serial_filt",
            namespace="/arduino/serial",
            output="screen",
        )
    ])