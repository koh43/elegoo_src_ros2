import os

import rclpy

from nodes.espcam import pub_cam_raw_info

def main(args=None):
    rclpy.init(args=args)

    params = {
        'node_name'  : 'pub_espcam_raw_info',
        'queue_size' : 5,
        'frame_id'   : 'espcam',
        'fps'        : 30,
        'calib_type' : 'matlab',
        'calib_path' : './espcam_calib_params.mat'
    }
    app = pub_cam_raw_info.PUB_CAM_RAW_INFO(params)

    rclpy.spin(app)
    app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()