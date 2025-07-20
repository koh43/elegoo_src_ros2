import rclpy

from nodes.espcam import pub_cam_rect

def main(args=None):
    rclpy.init(args=args)

    params = {
        'node_name'  : 'pub_espcam_rect',
        'queue_size' : 5,
    }
    app = pub_cam_rect.PUB_CAM_RECT(params)

    rclpy.spin(app)
    app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()