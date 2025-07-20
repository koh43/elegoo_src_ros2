import os

import rclpy

from nodes.espcam import pub_cam_vo

def main(args=None):
    rclpy.init(args=args)

    params = {
        'node_name'  : 'pub_espcam_vo',
        'queue_size' : 5,
        'calib_type' : 'matlab',
        'calib_path' : os.path.join(
            os.path.expanduser('~'),
            'espcam_calib',
            'espcam_calib_params.mat'
        )
    }
    app = pub_cam_vo.PUB_CAM_VO(params)

    rclpy.spin(app)
    app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()