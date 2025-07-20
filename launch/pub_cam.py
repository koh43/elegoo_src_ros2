import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="elegoo_src_ros2",
            executable="pub_cam_raw_info",
            namespace="/espcam",
            output="screen"
        ),
        launch_ros.actions.Node(
            package="elegoo_src_ros2",
            executable="pub_cam_raw",
            namespace="/espcam",
            output="screen"
        ),
        launch_ros.actions.Node(
            package="elegoo_src_ros2",
            executable="pub_cam_rect",
            namespace="/espcam",
            output="screen"
        ),
        # launch_ros.actions.Node(
        #     package="elegoo_src_ros2",
        #     executable="pub_cam_vo",
        #     namespace="/espcam",
        #     output="screen"
        # )
    ])