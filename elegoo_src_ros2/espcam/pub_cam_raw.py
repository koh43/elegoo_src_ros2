import rclpy

from nodes.espcam import pub_cam_raw

def main(args=None):
    rclpy.init(args=args)

    params = {
        'node_name'  : 'pub_espcam_raw',
        'queue_size' : 5,
        'cam_ip'     : "koh-esp32cam.local",
        'cam_port'   : 81,
        'frame_id'   : 'espcam',
        'fps'        : 30,
    }
    app = pub_cam_raw.PUB_ESPCAM_RAW(params)

    rclpy.spin(app)
    app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()